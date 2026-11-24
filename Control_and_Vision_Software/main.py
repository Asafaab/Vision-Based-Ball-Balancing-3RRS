import argparse
import csv
import logging
import math
import os
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)

from compensacion import CompensacionPitch, IMU_MAX_AGE_S
from controller import RobotController
from robotKinematics import RobotKinematics
from PID import PIDcontroller            # PID original (para comparar con --pid original)
from PID_mejorado import crear_pid       # PID mejorado v1 / v2
from camera import CameraModule, CENTER_X, CENTER_Y
from comunicacion import ComunicacionUART
from imu import MPU6050

# --- Ganancias del PID ORIGINAL (solo se usan con --pid original) ---
kp = 0.0063/3.5
ki = 0.00005/3
kd = 0.006025/4
alpha = 0.7
beta = 0.3

H = 8  # altura de la plataforma usada en el balanceo
SIGNO_X = 1
SIGNO_Y = -1

MAX_THETA = 14.0
PERDIDA_PELOTA_S = 0.12
PITCH_AZIMUT_DEFAULT = 180.0

CSV_COLUMNAS = ["t_s", "dt_s", "ball_x_px", "ball_y_px", "setpoint_x_px", "setpoint_y_px",
                "err_x_px", "err_y_px", "theta_deg", "phi_deg",
                "pwm1_us", "pwm2_us", "pwm3_us", "ang1_deg", "ang2_deg", "ang3_deg",
                "imu_roll_deg", "imu_pitch_deg", "pitch_comp_deg", "theta_rel_deg",
                "phi_rel_deg", "pid_recortado", "imu_valida", "pelota_detectada"]

# --- Frecuencia de envío UART hacia el myRIO (Hz) ---
UART_TX_HZ = 50          # Frecuencia máxima de POS hacia el myRIO.


def transformar_posicion(position):
    x, y = position

    x_robot = CENTER_X + SIGNO_X * (x - CENTER_X)
    y_robot = CENTER_Y + SIGNO_Y * (y - CENTER_Y)

    return x_robot, y_robot

def run_test_angles(a1, a2, a3):
    """Modo de prueba: fija los 3 angulos de los servos directamente y los
    mantiene, sin camara ni PID. Uso: python3 main.py --angles 0 0 0
    """
    model = RobotKinematics()
    robot = RobotController(model, model.lp, model.l1, model.l2, model.lb)
    print(f"Modo prueba: fijando angulos {a1}, {a2}, {a3} grados (Ctrl+C para salir)")
    robot.set_motor_angles(a1, a2, a3)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Saliendo...")
    finally:
        robot.shutdown()


def crear_controlador(nombre):
    if nombre == "original":
        return PIDcontroller(kp, ki, kd, alpha, beta, max_theta=MAX_THETA, conversion="tanh")
    return crear_pid(MAX_THETA, version=nombre)


# ─── Estados que se transmiten al myRIO ───────────────────────────────────────
ESTADO_SIN_CONTROL = "SIN_CONTROL"
ESTADO_CONTROLADO  = "CONTROLADO"
ESTADO_BUSCANDO    = "BUSCANDO"
ESTADO_REINICIANDO = "REINICIANDO"   # solo durante CMD:RESET (~1.5 s); si tu LabVIEW no lo
                                     # conoce, cambia esta constante a "SIN_CONTROL"

ESTADO_HEARTBEAT_S = 0.2     # el estado se REPITE cada 200 ms aunque no cambie
RESET_REANUDA_CONTROL = False  # False: tras RESET queda en SIN_CONTROL (esperando INICIAR)
                               # True : tras RESET, si estaba controlando, reanuda solo


class Emisor:
    """Telemetría Pi → myRIO con ritmo controlado y heartbeat de estado.

    - El ESTADO se manda al cambiar y además cada ESTADO_HEARTBEAT_S. Antes se mandaba
      solo al cambiar: si LabVIEW arrancaba/reiniciaba después, o se perdía esa línea,
      se quedaba con el último estado para siempre (p.ej. BUSCANDO).
    - POS se manda a uart_hz como máximo (no una vez por frame de cámara).
    """

    def __init__(self, comm, uart_hz):
        self.comm = comm
        self.periodo_tel = 1.0 / uart_hz
        self.estado = None
        self._t_estado = 0.0
        self._t_tel = 0.0

    def set_estado(self, estado, forzar=False):
        ahora = time.perf_counter()
        if forzar or estado != self.estado or ahora - self._t_estado >= ESTADO_HEARTBEAT_S:
            self.comm.enviar_estado(estado)
            self.estado = estado
            self._t_estado = ahora

    def posicion(self, pos):
        ahora = time.perf_counter()
        if ahora < self._t_tel:
            return
        self._t_tel = ahora + self.periodo_tel
        self.comm.enviar_posicion(*pos)

    def reiniciar(self):
        self.estado = None
        self._t_estado = self._t_tel = 0.0


class SesionLog:
    """Archivo CSV de una corrida de control."""

    def __init__(self, version, log_path):
        self.archivo = self.escritor = None
        self.n = 0
        self.t0 = self.t_prev = time.perf_counter()
        if log_path == "auto":
            os.makedirs("logs", exist_ok=True)
            self.ruta = os.path.join("logs", f"log_{version}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        else:
            self.ruta = log_path
        if self.ruta:
            self.archivo = open(self.ruta, "w", newline="")
            self.escritor = csv.writer(self.archivo)
            self.escritor.writerow(CSV_COLUMNAS)
            print(f"Guardando registro en {self.ruta}")

    def fila(self, position, target, theta, phi, robot, roll, pitch,
             pitch_comp, normal, limitada, imu_valida):
        ahora = time.perf_counter()
        if self.escritor is not None:
            pulsos = robot._calc_pulses(*robot.current_angles)
            nx, ny, nz = normal
            theta_rel = math.degrees(math.atan2(math.hypot(nx, ny), nz))
            phi_rel = math.degrees(math.atan2(ny, nx)) % 360
            if position is None:
                x, y, ex, ey = ("",) * 4
            else:
                x, y = [f"{valor:.3f}" for valor in position]
                ex, ey = [f"{p - r:.3f}" for p, r in zip(position, target)]
            self.escritor.writerow([
                f"{ahora - self.t0:.6f}", f"{ahora - self.t_prev:.6f}",
                x, y,
                f"{target[0]:.3f}", f"{target[1]:.3f}",
                ex, ey,
                f"{theta:.4f}", f"{phi:.4f}",
                *[f"{p:.0f}" for p in pulsos],
                *[f"{a:.4f}" for a in robot.current_angles],
                f"{roll:.3f}", f"{pitch:.3f}", f"{pitch_comp:.3f}",
                f"{theta_rel:.4f}", f"{phi_rel:.4f}",
                int(limitada), int(imu_valida), int(position is not None),
            ])
            self.n += 1
            if self.n % 500 == 0:
                self.archivo.flush()
        self.t_prev = ahora

    def cerrar(self):
        if self.archivo is not None:
            self.archivo.close()
            print(f"CSV guardado: {self.ruta}")
            self.archivo = self.escritor = None


def run_ball_balance(version="v1", duracion=None, log_path=None,
                     uart_port=None, uart_sim=False, uart_hz=UART_TX_HZ,
                     use_imu=True, imu_sim=False, auto_start=False,
                     pitch_azimut=PITCH_AZIMUT_DEFAULT, pitch_cero=0.0):
    """Bucle principal: UNA máquina de estados (ESPERA <-> CONTROL) sin bucles anidados.

    Comandos desde el myRIO (todos se procesan en cada vuelta, no uno por iteración):
      INICIAR  → ESPERA → CONTROL        PARAR → CONTROL → ESPERA
      RESET    → plataforma neutra, PID nuevo, setpoint al centro,
                 colas UART limpias; manda ACK:RESET_OK y vuelve a SIN_CONTROL.
      SETPOINT → cambia el objetivo.
    Estados transmitidos: SIN_CONTROL / CONTROLADO / BUSCANDO / REINICIANDO (+ heartbeat).
    """

    # ── UART ──────────────────────────────────────────────────────────────────
    comm = ComunicacionUART(port=uart_port or "/dev/ttyAMA0", modo_simulacion=uart_sim)
    comm.start()
    emisor = Emisor(comm, uart_hz)
    emisor.set_estado(ESTADO_SIN_CONTROL, forzar=True)
    print("UART iniciado. Esperando CMD:INICIAR desde el myRIO...")

    # Calibrar después de que se asiente el mecanismo y con el chasis inmóvil.
    robot = imu = None
    sesion = None
    try:
        model = RobotKinematics()
        robot = RobotController(model, model.lp, model.l1, model.l2, model.lb)
        robot.Goto_time_spherical(0, 0, H, t=0.5)
        time.sleep(0.4)                      # deja que la plataforma se asiente

        if use_imu:
            try:
                print("Calibrando giroscopio del MPU6050. Mantén el carrito inmóvil.")
                imu = MPU6050(cutoff_hz=0.7, sample_hz=100.0, modo_simulacion=imu_sim)
                if imu.simular and not imu_sim:
                    raise RuntimeError("Instala smbus2 para usar el MPU6050 real.")
                imu.start(nivelar_acelerometro=False)
                print("MPU6050 activo para compensación local de pitch.")
            except Exception as exc:
                print(f"Aviso: No se pudo iniciar el MPU6050 ({exc}). Continuando sin IMU.")
                imu = None

        target = [CENTER_X, CENTER_Y]
        pid = crear_controlador(version)
        modo = "ESPERA"
        t_ctrl0 = 0.0
        ultimo_theta = ultimo_phi = 0.0
        ultima_deteccion = -math.inf
        compensacion = CompensacionPitch(pitch_azimut, pitch_cero)
        emisor.set_estado(ESTADO_SIN_CONTROL, forzar=True)

        def iniciar():
            nonlocal pid, modo, sesion, t_ctrl0, ultimo_theta, ultimo_phi, ultima_deteccion
            print("CMD:INICIAR recibido. Arrancando control PID...")
            pid = crear_controlador(version)
            if sesion is not None:
                sesion.cerrar()
            sesion = SesionLog(version, log_path)
            robot.Goto_time_spherical(0, 0, H, t=0.5)
            t_ctrl0 = time.perf_counter()
            ultimo_theta = ultimo_phi = 0.0
            ultima_deteccion = -math.inf
            compensacion.reset()
            modo = "CONTROL"
            emisor.set_estado(ESTADO_BUSCANDO, forzar=True)   # pasa a CONTROLADO al ver la pelota

        def detener(msg, avisar=True):
            nonlocal modo, sesion
            print(msg)
            modo = "ESPERA"
            if sesion is not None:
                sesion.cerrar()
                sesion = None
            robot.Goto_time_spherical(0, 0, H, t=0.5)
            if avisar:
                emisor.set_estado(ESTADO_SIN_CONTROL, forzar=True)

        def reiniciar():
            nonlocal pid
            estaba_controlando = (modo == "CONTROL")
            print("CMD:RESET recibido. Reiniciando PID...")
            emisor.set_estado(ESTADO_REINICIANDO, forzar=True)
            detener("  (deteniendo control)", avisar=False)
            pid = crear_controlador(version)
            target[0], target[1] = CENTER_X, CENTER_Y
            comm.limpiar_tx()                        # fuera telemetría vieja
            emisor.reiniciar()
            comm.enviar_ack("RESET_OK")
            emisor.set_estado(ESTADO_SIN_CONTROL, forzar=True)
            print("Reset completo. Datos limpios.")
            if RESET_REANUDA_CONTROL and estaba_controlando:
                iniciar()

        with CameraModule() as camera:
            if auto_start:
                print("Auto-start activo: iniciando control PID de inmediato...")
                iniciar()

            while True:
                # ── 1) Comandos: se procesan TODOS los pendientes ──────────────
                for cmd in comm.leer_comandos():
                    c = cmd[0]
                    if c == "INICIAR" and modo == "ESPERA":
                        iniciar()
                    elif c == "PARAR" and modo == "CONTROL":
                        detener("CMD:PARAR recibido. Deteniendo control...")
                    elif c == "RESET":
                        reiniciar()
                        break                        # lo que venga detrás en esta tanda es viejo
                    elif c == "SETPOINT":
                        _, sp_x, sp_y = cmd
                        target[0], target[1] = sp_x, sp_y
                        print(f"CMD:SETPOINT → ({sp_x:.1f}, {sp_y:.1f})")
                        comm.enviar_ack(f"SETPOINT:{sp_x:.1f},{sp_y:.1f}")
                    elif c == "DESCONOCIDO":
                        print(f"RX desconocido: {cmd[1]!r}")

                # ── 2) Según el modo ───────────────────────────────────────────
                if modo == "ESPERA":
                    emisor.set_estado(ESTADO_SIN_CONTROL)        # heartbeat
                    time.sleep(0.005)
                    continue

                # modo == "CONTROL"
                if duracion is not None and time.perf_counter() - t_ctrl0 >= duracion:
                    detener(f"Se cumplieron {duracion:g} s. Terminando control.")
                    continue

                try:
                    data = camera.read(timeout=0.04)
                except TimeoutError:
                    data = None

                lectura = imu.get_data() if imu is not None else None
                ahora = time.perf_counter()
                roll_imu = lectura["roll"] if lectura else math.nan
                pitch_imu = lectura["pitch"] if lectura else math.nan
                t_imu = lectura["sample_time"] if lectura else None
                imu_valida = (t_imu is not None and ahora - t_imu <= IMU_MAX_AGE_S
                              and math.isfinite(pitch_imu))
                compensacion.actualizar(pitch_imu if imu_valida else None, ahora)

                position = None
                if data is not None and data.detected:
                    emisor.set_estado(ESTADO_CONTROLADO)
                    position = transformar_posicion(data.position)
                    theta, phi = pid.pid(tuple(target), position)
                    ultimo_theta, ultimo_phi = theta, phi
                    ultima_deteccion = ahora
                    emisor.posicion(position)
                else:
                    emisor.set_estado(ESTADO_BUSCANDO)
                    factor = max(0.0, 1.0 - (ahora - ultima_deteccion) / PERDIDA_PELOTA_S)
                    theta, phi = ultimo_theta * factor, ultimo_phi

                normal, limitada = compensacion.normal(theta, phi)
                robot.Goto_N_time_vector(*normal, H)
                sesion.fila(position, target, theta, phi, robot, roll_imu, pitch_imu,
                            compensacion.pitch_deg, normal, limitada, imu_valida)

    except KeyboardInterrupt:
        print("Saliendo (Ctrl+C)...")
    finally:
        if sesion is not None:
            sesion.cerrar()
        if imu is not None:
            imu.stop()
        if robot is not None:
            try:
                robot.Goto_time_spherical(0, 0, H, t=0.5)   # deja la plataforma plana
            except Exception:
                pass
            robot.shutdown()
        comm.stop()


def angulo_finito(texto):
    valor = float(texto)
    if not math.isfinite(valor):
        raise argparse.ArgumentTypeError("Se requiere un ángulo finito.")
    return valor


def parse_args():
    parser = argparse.ArgumentParser(description="Control de plataforma 3-RRS (Raspberry Pi 5)")
    parser.add_argument(
        "--angles", nargs=3, type=float, metavar=("T1", "T2", "T3"),
        help="Modo de prueba: fija los 3 angulos de los servos y sale del balanceo (ej: --angles 0 0 0)"
    )
    parser.add_argument(
        "--pid", choices=["v1", "v2", "original"], default="v1",
        help="PID a usar: v1 (por defecto), v2 u original"
    )
    parser.add_argument(
        "--dur", type=float, default=None,
        help="Duracion en segundos; al cumplirse se detiene solo (ej: --dur 60). Sin esto corre hasta Ctrl+C"
    )
    parser.add_argument(
        "--log", nargs="?", const="auto", default="auto", metavar="ARCHIVO",
        help="Archivo CSV del registro (por defecto se crea logs/log_<pid>_<fecha>.csv)"
    )
    parser.add_argument("--no-log", action="store_true", help="No guardar registro")
    # ── UART ──────────────────────────────────────────────────────────────────
    parser.add_argument(
        "--uart-port", default=None, metavar="PUERTO",
        help="Puerto UART (default: /dev/ttyAMA0). Ej: --uart-port /dev/ttyAMA2"
    )
    parser.add_argument(
        "--uart-sim", action="store_true",
        help="Modo simulacion UART: imprime mensajes por stdout en vez de usar el puerto serie"
    )
    parser.add_argument(
        "--uart-hz", type=float, default=UART_TX_HZ,
        help=f"Frecuencia de telemetría UART en Hz (default: {UART_TX_HZ} Hz hacia el myRIO)"
    )
    # ── IMU MPU6050 ──────────────────────────────────────────────────────────
    parser.add_argument(
        "--no-imu", action="store_true",
        help="Desactiva la lectura del sensor IMU MPU6050"
    )
    parser.add_argument(
        "--imu-sim", action="store_true",
        help="Modo simulacion para el sensor IMU MPU6050 (sin hardware I2C)"
    )
    parser.add_argument(
        "--pitch-azimut", type=angulo_finito, default=PITCH_AZIMUT_DEFAULT, metavar="GRADOS",
        help=f"Azimut de la normal del chasis para pitch IMU positivo (por defecto: {PITCH_AZIMUT_DEFAULT:g}°)"
    )
    parser.add_argument(
        "--pitch-cero", type=angulo_finito, default=0.0, metavar="GRADOS",
        help="Pitch medido por el MPU6050 con el chasis horizontal (se resta de la lectura)"
    )
    parser.add_argument(
        "--auto-start", action="store_true",
        help="Inicia el control PID y la cámara inmediatamente sin esperar CMD:INICIAR del myRIO"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.angles is not None:
        run_test_angles(*args.angles)
    else:
        ruta = None
        if not args.no_log:
            ruta = args.log   # "auto" o ruta explícita; _run_control_loop lo resuelve
        run_ball_balance(
            args.pid, args.dur, ruta,
            uart_port=args.uart_port,
            uart_sim=args.uart_sim,
            uart_hz=args.uart_hz,
            use_imu=not args.no_imu,
            imu_sim=args.imu_sim,
            auto_start=args.auto_start,
            pitch_azimut=args.pitch_azimut,
            pitch_cero=args.pitch_cero
        )
