#!/usr/bin/env python3
"""
imu.py – Módulo para MPU6050 en Raspberry Pi 5 con filtro complementario y calibración.

Características:
- Lectura I2C del MPU6050 (acelerómetro + giroscopio + temperatura).
- Calibración en reposo durante los primeros 200 ms. Se puede conservar
  la lectura absoluta del acelerómetro:
  * Resta 1.0 g en el eje Z para mantener el efecto de la gravedad en reposo (az ≈ 1.0 g).
  * Elimina offsets de X, Y y deriva del giroscopio.
- Fusión sensorial (Acelerómetro + Giroscopio) con Filtro Complementario:
  * Frecuencia de corte configurable, 1.5 Hz por defecto.
  * Factor alfa = tau / (tau + dt) dinámico según el dt de muestreo.
- Hilo en segundo plano a 100 Hz (dt = 10 ms) para cálculo de Roll y Pitch en tiempo real
  sin bloquear el ciclo de control ni el procesamiento de imagen.
"""

import math
import time
import threading
import logging

try:
    import smbus2 as smbus
    _SMBUS_DISPONIBLE = True
except ImportError:
    try:
        import smbus
        _SMBUS_DISPONIBLE = True
    except ImportError:
        _SMBUS_DISPONIBLE = False

logger = logging.getLogger(__name__)

# ─── Registros del MPU6050 ───────────────────────────────────────────────────
MPU_ADDR_DEFAULT  = 0x68   # Dirección I2C por defecto (AD0 = LOW)
REG_CONFIG        = 0x1A   # Configuración de DLPF digital
REG_GYRO_CONFIG   = 0x1B   # Escala del giroscopio
REG_ACCEL_CONFIG  = 0x1C   # Escala del acelerómetro
REG_ACCEL_XOUT_H  = 0x3B   # Registro inicial de datos (14 bytes contiguos)
REG_PWR_MGMT_1    = 0x6B   # Control de energía / despertar

# Factores de escala
# Rango +/- 2g  -> 16384 LSB/g
ACCEL_SCALE_2G    = 16384.0
# Rango +/- 250 deg/s -> 131.0 LSB / (deg/s)
GYRO_SCALE_250    = 131.0


class MPU6050:
    """Controlador y filtro complementario para el sensor IMU MPU6050."""

    def __init__(self, i2c_bus: int = 1, address: int = MPU_ADDR_DEFAULT,
                 cutoff_hz: float = 1.5, sample_hz: float = 100.0,
                 modo_simulacion: bool = False):
        """
        Parameters
        ----------
        i2c_bus         : Número del bus I2C (en Pi 5 suele ser 1: GPIO 2 SDA / GPIO 3 SCL)
        address         : Dirección I2C del sensor (0x68 ó 0x69)
        cutoff_hz       : Frecuencia de corte del filtro pasabajas (por defecto 1.5 Hz)
        sample_hz       : Frecuencia de muestreo del hilo interno (por defecto 100 Hz = 10 ms)
        modo_simulacion : Si True o si no hay smbus, simula lecturas en reposo
        """
        self.bus_num = i2c_bus
        self.address = address
        self.cutoff_hz = cutoff_hz
        self.sample_hz = sample_hz
        self.sample_interval = 1.0 / sample_hz
        self.simular = modo_simulacion or not _SMBUS_DISPONIBLE

        # Constante de tiempo del filtro: tau = 1 / (2 * pi * fc)
        self.tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)

        # Offsets de calibración
        self.offset_ax = 0.0
        self.offset_ay = 0.0
        self.offset_az = 0.0
        self.offset_gx = 0.0
        self.offset_gy = 0.0
        self.offset_gz = 0.0

        # Estados filtrados (ángulos en grados)
        self.roll = 0.0
        self.pitch = 0.0
        self.temp_c = 0.0

        # Datos instantáneos corregidos
        self.ax = 0.0
        self.ay = 0.0
        self.az = 1.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        self.calibrado = False
        self._bus = None
        self._lock = threading.Lock()
        self._io_lock = threading.Lock()   # serializa lecturas del hilo vs recalibración
        self._err_consecutivos = 0         # errores I2C seguidos (para auto-recuperar)
        self._stop_event = threading.Event()
        self._thread = None
        self._last_time = None
        self._sample_time = None
        self._retry_at = 0.0

    # ──────────────────────────────────────────────────────────────────────────
    # Inicialización de Hardware
    # ──────────────────────────────────────────────────────────────────────────

    def init_sensor(self) -> bool:
        """Configura los registros del MPU6050 vía I2C."""
        if self.simular:
            logger.warning("MPU6050 en MODO SIMULACIÓN (sin hardware real I2C)")
            return True

        if not _SMBUS_DISPONIBLE:
            logger.error("smbus/smbus2 no está instalado. Ejecuta: pip install smbus2")
            return False

        try:
            self._bus = smbus.SMBus(self.bus_num)

            # 1. Despertar MPU6050 (PWR_MGMT_1 = 0x00)
            self._bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x00)
            time.sleep(0.01)

            # 2. Configurar filtro pasabajas interno de hardware DLPF a 42 Hz (0x03)
            # Esto ayuda a rechazar vibraciones mecánicas de alta frecuencia
            self._bus.write_byte_data(self.address, REG_CONFIG, 0x03)

            # 3. Configurar escala de giroscopio a +/- 250 °/s (0x00)
            self._bus.write_byte_data(self.address, REG_GYRO_CONFIG, 0x00)

            # 4. Configurar escala de acelerómetro a +/- 2g (0x00)
            self._bus.write_byte_data(self.address, REG_ACCEL_CONFIG, 0x00)

            logger.info("MPU6050 inicializado correctamente en I2C bus %d, addr 0x%02X",
                        self.bus_num, self.address)
            return True
        except Exception as exc:
            logger.error("Error al inicializar MPU6050: %s", exc)
            if self._bus is not None:
                try:
                    self._bus.close()
                except OSError:
                    pass
                self._bus = None
            return False

    # ──────────────────────────────────────────────────────────────────────────
    # Lectura de datos crudos
    # ──────────────────────────────────────────────────────────────────────────

    def _read_raw_data(self):
        """Lee en ráfaga 14 bytes: 6 de acelerómetro, 2 de temp y 6 de giroscopio."""
        if self.simular:
            # En simulación devolvemos sensor en reposo horizontal
            return 0.0, 0.0, 1.0, 25.0, 0.0, 0.0, 0.0

        if self._bus is None:
            if time.perf_counter() < self._retry_at or not self._reiniciar_bus():
                return None

        try:
            # Lectura en bloque I2C de 14 bytes desde 0x3B
            data = self._bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)

            def to_int16(h, l):
                val = (h << 8) | l
                return val - 65536 if val > 32767 else val

            raw_ax = to_int16(data[0], data[1])
            raw_ay = to_int16(data[2], data[3])
            raw_az = to_int16(data[4], data[5])
            raw_temp = to_int16(data[6], data[7])
            raw_gx = to_int16(data[8], data[9])
            raw_gy = to_int16(data[10], data[11])
            raw_gz = to_int16(data[12], data[13])

            ax = raw_ax / ACCEL_SCALE_2G
            ay = raw_ay / ACCEL_SCALE_2G
            az = raw_az / ACCEL_SCALE_2G
            temp_c = (raw_temp / 340.0) + 36.53
            gx = raw_gx / GYRO_SCALE_250
            gy = raw_gy / GYRO_SCALE_250
            gz = raw_gz / GYRO_SCALE_250

            self._err_consecutivos = 0
            return ax, ay, az, temp_c, gx, gy, gz
        except Exception as exc:
            self._err_consecutivos += 1
            logger.warning("Error leyendo MPU6050 (%d seguidos): %s",
                           self._err_consecutivos, exc)
            # Bus I2C/sensor colgado: tras 5 fallos seguidos se reinicia el sensor
            # (antes solo se arreglaba matando y relanzando el programa).
            if self._err_consecutivos == 5:
                self._reiniciar_bus()
            return None

    def _reiniciar_bus(self):
        """Cierra y reabre el bus I2C y reconfigura el MPU6050."""
        if self.simular:
            return True
        logger.warning("Reiniciando bus I2C / MPU6050...")
        try:
            if self._bus is not None:
                self._bus.close()
        except Exception:
            pass
        self._bus = None
        self._retry_at = time.perf_counter() + 1.0
        self._err_consecutivos = 0
        return self.init_sensor()

    # ──────────────────────────────────────────────────────────────────────────
    # Calibración
    # ──────────────────────────────────────────────────────────────────────────

    def calibrate(self, duration_s: float = 0.20, sample_interval_s: float = 0.005,
                  nivelar_acelerometro: bool = True):
        """
        Calibra el sensor recogiendo datos durante los primeros ~200 ms en reposo.
        
        Importante:
        - Siempre se calibra el sesgo del giroscopio.
        - nivelar_acelerometro=False conserva las lecturas de gravedad aunque el chasis
          se encuentre en una pendiente. El IMU debe estar inmóvil.
        - Al nivelar el acelerómetro, en Z se resta 1.0 g para conservar la gravedad
          en reposo (offset_z = mean_az - 1.0 -> az_corregido = az - offset_z ≈ 1.0 g).
        - Inicializa los ángulos Roll y Pitch directamente con los del acelerómetro
          para evitar cualquier salto brusco inicial en el filtro.
        """
        logger.info("Iniciando calibración del MPU6050 (duración: %.3f s)...", duration_s)
        t_start = time.perf_counter()
        samples = 0

        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0
        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0

        while (time.perf_counter() - t_start) < duration_s:
            lectura = self._read_raw_data()
            if lectura is None:
                time.sleep(sample_interval_s)
                continue
            ax, ay, az, _, gx, gy, gz = lectura
            sum_ax += ax
            sum_ay += ay
            sum_az += az
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            samples += 1
            time.sleep(sample_interval_s)

        if samples > 0:
            mean_ax = sum_ax / samples
            mean_ay = sum_ay / samples
            mean_az = sum_az / samples
            mean_gx = sum_gx / samples
            mean_gy = sum_gy / samples
            mean_gz = sum_gz / samples

            with self._lock:
                self.offset_ax = mean_ax if nivelar_acelerometro else 0.0
                self.offset_ay = mean_ay if nivelar_acelerometro else 0.0
                self.offset_az = mean_az - 1.0 if nivelar_acelerometro else 0.0
                self.offset_gx = mean_gx
                self.offset_gy = mean_gy
                self.offset_gz = mean_gz

                # Inicializar roll y pitch en base a la gravedad calibrada
                init_ax = mean_ax - self.offset_ax
                init_ay = mean_ay - self.offset_ay
                init_az = mean_az - self.offset_az

                roll_acc = math.atan2(init_ay, init_az) * 180.0 / math.pi
                pitch_acc = math.atan2(-init_ax, math.sqrt(init_ay**2 + init_az**2)) * 180.0 / math.pi

                self.roll = roll_acc
                self.pitch = pitch_acc
                self.calibrado = True
                self._sample_time = time.perf_counter()

            logger.info("Calibración completada con %d muestras:", samples)
            logger.info("  Offset Accel: X=%.4f g, Y=%.4f g, Z=%.4f g",
                        self.offset_ax, self.offset_ay, self.offset_az)
            logger.info("  Offset Gyro : X=%.4f °/s, Y=%.4f °/s, Z=%.4f °/s",
                        self.offset_gx, self.offset_gy, self.offset_gz)
            logger.info("  Ángulos iniciales: Roll=%.2f°, Pitch=%.2f°",
                        self.roll, self.pitch)
        else:
            raise RuntimeError("No se pudieron tomar muestras durante la calibración del MPU6050.")

    # ──────────────────────────────────────────────────────────────────────────
    # Actualización del Filtro Complementario (Paso a paso)
    # ──────────────────────────────────────────────────────────────────────────

    def update(self) -> tuple[float, float]:
        """
        Realiza una lectura, aplica corrección de offset y fusiona acelerómetro
        y giroscopio con el filtro complementario.

        Returns
        -------
        (roll, pitch) en grados
        """
        now = time.perf_counter()
        if self._last_time is None:
            self._last_time = now
            dt = self.sample_interval
        else:
            dt = now - self._last_time
            self._last_time = now

        if dt <= 0.0 or dt > 0.5:
            dt = self.sample_interval

        # Lectura de datos crudos
        lectura = self._read_raw_data()
        if lectura is None:
            return self.get_angles()
        raw_ax, raw_ay, raw_az, temp_c, raw_gx, raw_gy, raw_gz = lectura

        with self._lock:
            # Corrección de offset
            self.ax = raw_ax - self.offset_ax
            self.ay = raw_ay - self.offset_ay
            self.az = raw_az - self.offset_az
            self.gx = raw_gx - self.offset_gx
            self.gy = raw_gy - self.offset_gy
            self.gz = raw_gz - self.offset_gz
            self.temp_c = temp_c

            if not self.calibrado:
                # Si aún no está calibrado, reportamos 0.0
                return 0.0, 0.0

            # 1. Ángulos calculados únicamente con el acelerómetro (grados)
            # Roll (inclinación lateral en X)
            roll_acc = math.atan2(self.ay, self.az) * 180.0 / math.pi
            # Pitch (inclinación longitudinal en Y)
            pitch_acc = math.atan2(-self.ax, math.sqrt(self.ay**2 + self.az**2)) * 180.0 / math.pi

            # 2. Factor de filtro complementario con dt medido.
            alpha = self.tau / (self.tau + dt)

            # 3. Fusión sensorial:
            # - Integra la velocidad angular del giroscopio (alta frecuencia, respuesta rápida)
            # - Corrige con el acelerómetro a través del filtro pasabajas (baja frecuencia, sin deriva)
            self.roll = alpha * (self.roll + self.gx * dt) + (1.0 - alpha) * roll_acc
            self.pitch = alpha * (self.pitch + self.gy * dt) + (1.0 - alpha) * pitch_acc

            self._sample_time = now
            return self.roll, self.pitch

    # ──────────────────────────────────────────────────────────────────────────
    # Hilo en segundo plano para muestreo a alta frecuencia (10 ms / 100 Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def start(self, auto_calibrate: bool = True, nivelar_acelerometro: bool = True):
        """Inicia el sensor, calibra los primeros 200 ms y lanza el hilo a 100 Hz."""
        try:
            if not self.init_sensor():
                raise RuntimeError("No se pudo inicializar el MPU6050.")
            if auto_calibrate:
                self.calibrate(duration_s=0.20, nivelar_acelerometro=nivelar_acelerometro)
        except BaseException:
            self.stop()
            raise

        self._stop_event.clear()
        self._last_time = time.perf_counter()
        self._thread = threading.Thread(target=self._worker_loop, name="mpu6050-loop", daemon=True)
        self._thread.start()
        logger.info("Hilo MPU6050 iniciado a %.1f Hz (intervalo %.1f ms)",
                    self.sample_hz, self.sample_interval * 1000.0)
        return self

    def stop(self):
        """Detiene el hilo de muestreo y cierra la conexión I2C."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
        logger.info("MPU6050 detenido.")

    def recalibrar(self, duration_s: float = 0.20):
        """Re-calibra el IMU en caliente (llamar con la plataforma plana y quieta).

        Pausa el hilo de muestreo mientras dura, reconfigura el sensor, recalcula
        offsets y deja Roll/Pitch en ~0 sin saltos en el filtro complementario.
        """
        with self._io_lock:
            self._reiniciar_bus()          # reconfigura registros (recupera sensor colgado)
            self.calibrate(duration_s=duration_s)
            self._last_time = time.perf_counter()   # evita un dt enorme en el 1.er update
        logger.info("IMU recalibrado: Roll=%.2f° Pitch=%.2f°", self.roll, self.pitch)

    def _worker_loop(self):
        """Bucle del hilo que corre periódicamente cada 10 ms."""
        while not self._stop_event.is_set():
            t_cycle_start = time.perf_counter()
            with self._io_lock:
                self.update()
            elapsed = time.perf_counter() - t_cycle_start
            sleep_time = self.sample_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ──────────────────────────────────────────────────────────────────────────
    # Getters públicos seguros para hilos
    # ──────────────────────────────────────────────────────────────────────────

    def get_angles(self) -> tuple[float, float]:
        """Devuelve los ángulos actuales (roll, pitch) en grados."""
        with self._lock:
            return self.roll, self.pitch

    def get_data(self) -> dict:
        """Devuelve un diccionario completo con aceleraciones, giro, temp y ángulos."""
        with self._lock:
            return {
                "roll": self.roll,
                "pitch": self.pitch,
                "ax": self.ax,
                "ay": self.ay,
                "az": self.az,
                "gx": self.gx,
                "gy": self.gy,
                "gz": self.gz,
                "temp_c": self.temp_c,
                "calibrado": self.calibrado,
                "sample_time": self._sample_time
            }


# ─── Bloque de prueba autónomo ───────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    print("=== Test de MPU6050 para Raspberry Pi 5 ===")
    print("Iniciando MPU6050 (calibrando primeros 200 ms)...")

    imu = MPU6050(cutoff_hz=1.5, sample_hz=100.0)
    imu.start(auto_calibrate=True)

    try:
        t_start = time.perf_counter()
        while time.perf_counter() - t_start < 5.0:
            roll, pitch = imu.get_angles()
            data = imu.get_data()
            print(f"\rRoll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | "
                  f"Ax: {data['ax']:5.2f}g Ay: {data['ay']:5.2f}g Az: {data['az']:5.2f}g | "
                  f"Temp: {data['temp_c']:.1f}°C", end="", flush=True)
            time.sleep(0.05)
        print("\nPrueba completada exitosamente.")
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        imu.stop()
