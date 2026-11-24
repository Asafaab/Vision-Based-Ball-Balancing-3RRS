#!/usr/bin/env python3
"""
comunicacion.py  –  Módulo de comunicación UART entre Raspberry Pi 5 y myRIO.

Raspberry Pi 5  <──TX/RX──>  myRIO (UART1 en conector MXP o MSP)

Protocolo ASCII, 115200 bps, 8N1, terminador '\\n'.

──────────────────────────────────────────────────────────────────
  PI  →  MYRIO  (mensajes de telemetría, ~10 Hz)
──────────────────────────────────────────────────────────────────
  POS:<x>,<y>\\n          Posición actual de la pelota en píxeles
                           x: 0..1280,  y: 0..800
                           Ejemplo:  POS:645.3,392.8\\n

  ESTADO:<estado>\\n      Estado del control
                           CONTROLADO   – PID activo, pelota bajo control
                           SIN_CONTROL  – PID detenido (en espera)
                           BUSCANDO     – cámara no detecta la pelota
                           Ejemplo:  ESTADO:CONTROLADO\\n

  ERR:<ex>,<ey>\\n        Error de posición respecto al setpoint (píxeles)
                           Ejemplo:  ERR:-12.4,5.1\\n

  DATO:<t>,<x>,<y>,<ex>,<ey>,<theta>,<phi>\\n
                           Fila de telemetría completa (igual que el CSV)
                           t: tiempo en segundos desde el inicio del control
                           Ejemplo:  DATO:12.340,645.3,392.8,-12.4,5.1,3.21,135.7\\n

──────────────────────────────────────────────────────────────────
  MYRIO  →  PI  (comandos de control)
──────────────────────────────────────────────────────────────────
  CMD:INICIAR\\n          Inicia (o reanuda) el bucle de control PID
  CMD:PARAR\\n            Detiene el control y guarda el CSV hasta ese momento
  CMD:RESET\\n            Reinicia el PID (integral y derivada a cero)
  CMD:SETPOINT:<x>,<y>\\n Cambia el setpoint (píxeles). Ej: CMD:SETPOINT:640,400\\n
  CMD:PING\\n             Solicita confirmación de vida  →  Pi responde ACK:PONG\\n

──────────────────────────────────────────────────────────────────
Uso desde main.py
──────────────────────────────────────────────────────────────────
  comm = ComunicacionUART()
  comm.start()

  # enviar datos
  comm.enviar_posicion(x, y)
  comm.enviar_estado("CONTROLADO")
  comm.enviar_error(ex, ey)
  comm.enviar_dato(t, x, y, ex, ey, theta, phi)

  # leer comando pendiente (no bloqueante)
  cmd = comm.leer_comando()   # None si no hay nada
  if cmd and cmd[0] == "INICIAR":
      ...

  comm.stop()
"""

import queue
import threading
import time
import logging

try:
    import serial
    _SERIAL_DISPONIBLE = True
except ImportError:
    _SERIAL_DISPONIBLE = False

logger = logging.getLogger(__name__)

# ─── Parámetros de puerto (ajusta si usas otro UART de la Pi 5) ───────────────
UART_PORT    = "/dev/ttyAMA0"   # GPIO 14 (TX) / GPIO 15 (RX)  –  pins 8 / 10
# /dev/serial0 otra opción, pero depende de la configuración del sistema operativo
UART_BAUD    = 115200
UART_TIMEOUT = 0.05             # s – timeout de lectura no bloqueante


class ComunicacionUART:
    """Comunicación bidireccional UART con el myRIO.

    Hilo de TX: envía mensajes de una cola (no bloquea el bucle de control).
    Hilo de RX: lee líneas y deposita comandos en otra cola.
    """

    def __init__(self, port=UART_PORT, baud=UART_BAUD,
                 tx_queue_size=100, modo_simulacion=False):
        """
        Parameters
        ----------
        port            : Puerto serie, p.ej. '/dev/ttyAMA0'
        baud            : Velocidad en baudios
        tx_queue_size   : Máximo de mensajes pendientes en la cola TX
                          (los más antiguos se descartan si se llena)
        modo_simulacion : Si True, no abre el puerto serie (útil para pruebas
                          en PC sin hardware). Los mensajes se imprimen por stdout.
        """
        self.port    = port
        self.baud    = baud
        self.simular = modo_simulacion or not _SERIAL_DISPONIBLE

        self._tx_queue  = queue.Queue(maxsize=tx_queue_size)
        self._cmd_queue = queue.Queue(maxsize=50)  # comandos del myRIO (acotada: si LabVIEW
                                                   # manda a 100 Hz no crece sin límite)
        self._rx_buffer = b""                      # bytes recibidos aún sin '\n'
        self._rx_lock   = threading.Lock()

        self._ser       = None
        self._stop_ev   = threading.Event()
        self._tx_thread = None
        self._rx_thread = None
        self._lock      = threading.Lock()       # para acceso seguro al puerto

        # Estadísticas (opcionales, útiles para debug)
        self.stats = {"tx_enviados": 0, "tx_descartados": 0,
                      "rx_recibidos": 0, "rx_errores": 0,
                      "rx_basura": 0, "reaperturas": 0}

    # ──────────────────────────────────────────────────────────────────────────
    # Ciclo de vida
    # ──────────────────────────────────────────────────────────────────────────

    def start(self):
        """Abre el puerto y lanza los hilos TX/RX."""
        if not self.simular:
            if not _SERIAL_DISPONIBLE:
                raise ImportError(
                    "Instala pyserial:  pip install pyserial")
            self._ser = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=UART_TIMEOUT,
                write_timeout=0.5          # que un TX atascado nunca congele el hilo
            )
            self._ser.reset_input_buffer()   # descarta basura previa del puerto
            self._ser.reset_output_buffer()
            logger.info("UART abierto: %s @ %d bps", self.port, self.baud)
        else:
            logger.warning("UART en MODO SIMULACIÓN (sin hardware real)")

        self._stop_ev.clear()
        self._tx_thread = threading.Thread(
            target=self._worker_tx, name="uart-tx", daemon=True)
        self._rx_thread = threading.Thread(
            target=self._worker_rx, name="uart-rx", daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()
        return self

    def stop(self):
        """Detiene los hilos y cierra el puerto."""
        self._stop_ev.set()
        # Desbloquear worker TX
        try:
            self._tx_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._tx_thread:
            self._tx_thread.join(timeout=2.0)
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        logger.info("UART cerrado. Stats: %s", self.stats)

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()

    # ──────────────────────────────────────────────────────────────────────────
    # API pública – ENVÍO (Pi → myRIO)
    # ──────────────────────────────────────────────────────────────────────────

    def enviar_posicion(self, x: float, y: float):
        """Envía la posición actual de la pelota en píxeles."""
        self._encolar(f"POS:{x:.1f},{y:.1f}")

    def enviar_estado(self, estado: str):
        """Envía el estado del control.
        estado: 'CONTROLADO' | 'SIN_CONTROL' | 'BUSCANDO'
        """
        self._encolar(f"ESTADO:{estado}")

    def enviar_error(self, ex: float, ey: float):
        """Envía el error de posición respecto al setpoint."""
        self._encolar(f"ERR:{ex:.1f},{ey:.1f}")

    def enviar_dato(self, t: float, x: float, y: float,
                    ex: float, ey: float, theta: float, phi: float):
        """Envía una fila de telemetría completa (equivalente al CSV)."""
        self._encolar(
            f"DATO:{t:.3f},{x:.1f},{y:.1f},{ex:.1f},{ey:.1f},"
            f"{theta:.3f},{phi:.3f}"
        )

    def enviar_ack(self, texto: str = "OK"):
        """Envía un ACK genérico al myRIO."""
        self._encolar(f"ACK:{texto}")

    # ──────────────────────────────────────────────────────────────────────────
    # API pública – RECEPCIÓN (myRIO → Pi)
    # ──────────────────────────────────────────────────────────────────────────

    def leer_comando(self):
        """Devuelve el próximo comando pendiente o None si no hay.

        Returns
        -------
        None  si no hay comandos en cola.
        ("INICIAR",)                              para CMD:INICIAR
        ("PARAR",)                                para CMD:PARAR
        ("RESET",)                                para CMD:RESET
        ("PING",)                                 para CMD:PING
        ("SETPOINT", x_float, y_float)            para CMD:SETPOINT:x,y
        ("DESCONOCIDO", linea_raw)                para cualquier otra cosa
        """
        try:
            return self._cmd_queue.get_nowait()
        except queue.Empty:
            return None

    def leer_comandos(self):
        """Vacía la cola y devuelve TODOS los comandos pendientes (lista, en orden).

        Usar esto en vez de leer_comando() en bucles lentos: leer un solo comando por
        iteración hace que, si LabVIEW manda más rápido de lo que la Pi consume, los
        comandos se acumulen y lleguen con retraso (parece que "se pegó").
        """
        cmds = []
        while True:
            try:
                cmds.append(self._cmd_queue.get_nowait())
            except queue.Empty:
                return cmds

    def limpiar_rx(self):
        """Descarta comandos pendientes, bytes parciales y el buffer de entrada del puerto."""
        with self._rx_lock:
            self._rx_buffer = b""
        while True:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                break
        if self._ser is not None and not self.simular:
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass

    def limpiar_tx(self):
        """Descarta la telemetría vieja pendiente de enviar (para mandar datos limpios)."""
        while True:
            try:
                self._tx_queue.get_nowait()
            except queue.Empty:
                break
        if self._ser is not None and not self.simular:
            try:
                self._ser.reset_output_buffer()
            except Exception:
                pass

    def hay_comandos(self) -> bool:
        """True si hay al menos un comando pendiente."""
        return not self._cmd_queue.empty()

    # ──────────────────────────────────────────────────────────────────────────
    # Internos
    # ──────────────────────────────────────────────────────────────────────────

    def _encolar(self, mensaje: str):
        """Pone un mensaje en la cola TX. Si está llena, descarta el más viejo."""
        linea = (mensaje + "\n").encode("ascii")
        try:
            self._tx_queue.put_nowait(linea)
        except queue.Full:
            # La cola está llena: descartamos el mensaje más viejo y metemos el nuevo
            try:
                self._tx_queue.get_nowait()
                self.stats["tx_descartados"] += 1
            except queue.Empty:
                pass
            try:
                self._tx_queue.put_nowait(linea)
            except queue.Full:
                self.stats["tx_descartados"] += 1

    def _put_cmd(self, cmd):
        """Mete un comando en la cola; si está llena descarta el más viejo."""
        try:
            self._cmd_queue.put_nowait(cmd)
        except queue.Full:
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._cmd_queue.put_nowait(cmd)
            except queue.Full:
                pass

    def _reabrir(self):
        """Cierra y vuelve a abrir el puerto (recuperación ante error del UART)."""
        try:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            time.sleep(0.2)
            self._ser = serial.Serial(
                self.port, self.baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=UART_TIMEOUT, write_timeout=0.5)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            with self._rx_lock:
                self._rx_buffer = b""
            self.stats["reaperturas"] += 1
            logger.warning("UART reabierto")
            return True
        except Exception as exc:
            logger.error("No se pudo reabrir UART: %s", exc)
            return False

    def _worker_tx(self):
        """Hilo dedicado a enviar mensajes de la cola TX."""
        while not self._stop_ev.is_set():
            try:
                linea = self._tx_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if linea is None:          # señal de parada
                break
            if self.simular:
                print(f"[UART-TX] {linea.decode().strip()}")
                self.stats["tx_enviados"] += 1
            else:
                try:
                    with self._lock:
                        self._ser.write(linea)
                    self.stats["tx_enviados"] += 1
                except Exception as exc:
                    logger.error("Error TX UART: %s", exc)
                    with self._lock:
                        self._reabrir()

    def _worker_rx(self):
        """Hilo dedicado a leer líneas del myRIO (resincroniza solo ante basura)."""
        while not self._stop_ev.is_set():
            if self.simular:
                time.sleep(0.1)
                continue
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if not chunk:
                    continue
                lineas = []
                with self._rx_lock:
                    self._rx_buffer += chunk
                    while b"\n" in self._rx_buffer:
                        linea, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
                        lineas.append(linea)
                    # Sin '\n' y demasiado largo = basura (p.ej. ruido al abrir/cerrar
                    # VISA en LabVIEW). Se descarta para no pegarla al próximo comando.
                    if len(self._rx_buffer) > 128:
                        self._rx_buffer = b""
                        self.stats["rx_basura"] += 1
                for linea in lineas:
                    self._procesar_rx(linea.decode("ascii", errors="replace").strip())
            except Exception as exc:
                if not self._stop_ev.is_set():
                    logger.error("Error RX UART: %s", exc)
                    self.stats["rx_errores"] += 1
                    time.sleep(0.05)
                    with self._lock:
                        self._reabrir()

    def _procesar_rx(self, linea: str):
        """Parsea una línea recibida y deposita el comando en la cola."""
        if not linea:
            return
        # Tolera basura delante del comando (p.ej. '\x00CMD:INICIAR'): se queda con
        # lo que hay desde la ÚLTIMA aparición de 'CMD:'.
        idx = linea.rfind("CMD:")
        if idx < 0:
            self.stats["rx_basura"] += 1
            logger.warning("Mensaje RX sin CMD: %r", linea)
            return
        linea = linea[idx:]
        self.stats["rx_recibidos"] += 1
        logger.info("[UART-RX] %s", linea)

        cuerpo = linea[4:].strip()   # quita "CMD:"

        if cuerpo == "INICIAR":
            self._put_cmd(("INICIAR",))

        elif cuerpo == "PARAR":
            self._put_cmd(("PARAR",))

        elif cuerpo == "RESET":
            self._put_cmd(("RESET",))

        elif cuerpo == "PING":
            self.enviar_ack("PONG")
            self._put_cmd(("PING",))

        elif cuerpo.startswith("SETPOINT:"):
            try:
                partes = cuerpo[9:].split(",")
                sp_x, sp_y = float(partes[0]), float(partes[1])
                self._put_cmd(("SETPOINT", sp_x, sp_y))
            except (ValueError, IndexError):
                logger.warning("SETPOINT malformado: %r", linea)
                self._put_cmd(("DESCONOCIDO", linea))

        else:
            logger.warning("Comando desconocido: %r", linea)
            self._put_cmd(("DESCONOCIDO", linea))


# ─── Bloque de prueba autónoma (sin myRIO) ────────────────────────────────────
if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.DEBUG,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    print("=== Test de ComunicacionUART (modo simulación) ===")
    comm = ComunicacionUART(modo_simulacion=True)
    comm.start()

    t0 = time.perf_counter()
    for i in range(20):
        t = time.perf_counter() - t0
        comm.enviar_posicion(640 + i * 2, 400 - i)
        comm.enviar_estado("CONTROLADO")
        comm.enviar_error(-12.4 + i, 5.1)
        comm.enviar_dato(t, 640 + i * 2, 400 - i, -12.4 + i, 5.1, 3.2, 135.7)
        time.sleep(0.1)

    comm.stop()
    print("Stats:", comm.stats)
    print("=== Fin del test ===")
