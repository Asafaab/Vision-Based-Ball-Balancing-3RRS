import argparse
import fcntl
import math
import os
from pathlib import Path
import threading
import time


PULSE_MIN_US = 500.0
PULSE_MAX_US = 2500.0
OFFSETS = (5.0, 0.0, 7.0)


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def angle_to_pulse_us(angle_deg):
    """Convierte el angulo fisico de 0 a 180 grados a microsegundos."""
    if not math.isfinite(angle_deg):
        raise ValueError("El angulo debe ser finito.")
    angle_deg = clamp(angle_deg, 0.0, 180.0)
    return PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * angle_deg / 180.0


def validate_angles(angles):
    angles = tuple(float(angle) for angle in angles)
    if len(angles) != 3 or any(
        not math.isfinite(angle) or not -10 <= angle <= 80
        for angle in angles
    ):
        raise ValueError("Ingresa tres angulos entre -5 y 65 grados.")
    return angles


class ServoPWM:
    """Controlador PWM de bajo nivel por hardware para los 3 canales de la Raspberry Pi 5."""
    CHANNELS = (0, 1, 2)
    PERIOD_NS = 20_000_000  # 50 Hz -> periodo de 20 ms

    @staticmethod
    def find_chip():
        symbol = Path('/proc/device-tree/__symbols__/rp1_pwm0')
        if not symbol.exists():
            raise RuntimeError('No se encontro RP1 PWM0. Este modulo requiere Raspberry Pi 5.')
        node = Path('/proc/device-tree' + symbol.read_text().rstrip('\x00')).resolve()
        chips = [
            p for p in Path('/sys/class/pwm').glob('pwmchip*')
            if (p / 'device/of_node').resolve() == node
        ]
        if len(chips) != 1:
            raise RuntimeError('Activa el overlay balancer-pwm-pi5 y reinicia la Raspberry.')
        if int((chips[0] / 'npwm').read_text()) < 3:
            raise RuntimeError('El controlador no expone los tres canales requeridos.')
        return chips[0]

    def __init__(self, initial_us=(1500, 1500, 1500), timeout_s=None):
        self._validate(initial_us)
        self.chip = self.find_chip()
        self.paths, self.fds = [], []
        self.current = None
        self.neutral = tuple(initial_us)
        self.timeout_s = timeout_s
        self.error = None
        self.tripped = threading.Event()
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.lock_file = open('/run/lock/ball_balancer_pwm.lock', 'a')

        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            for channel in self.CHANNELS:
                if (self.chip / f'pwm{channel}').exists():
                    raise RuntimeError(f'PWM {channel} ya exportado: revisa que no lo use otro programa.')

            for channel in self.CHANNELS:
                path = self.chip / f'pwm{channel}'
                (self.chip / 'export').write_text(str(channel))
                self.paths.append(path)
                deadline = time.monotonic() + 1.0
                while not (path / 'enable').exists():
                    if time.monotonic() > deadline:
                        raise RuntimeError(f'No se pudo exportar PWM {channel}.')
                    time.sleep(0.005)

                (path / 'enable').write_text('0')
                (path / 'duty_cycle').write_text('0')
                (path / 'polarity').write_text('normal')
                (path / 'period').write_text(str(self.PERIOD_NS))
                self.fds.append(os.open(path / 'duty_cycle', os.O_WRONLY))

            self._apply(initial_us)
            for path in self.paths:
                (path / 'enable').write_text('1')

            self.last_update = time.monotonic()
            if timeout_s is not None:
                if not math.isfinite(timeout_s) or timeout_s <= 0:
                    raise ValueError('El timeout debe ser positivo.')
                self.thread = threading.Thread(target=self._watchdog, daemon=True)
                self.thread.start()

        except BaseException:
            self.close()
            raise

    @staticmethod
    def _validate(pulses):
        if len(pulses) != 3 or any(not math.isfinite(p) or not PULSE_MIN_US <= p <= PULSE_MAX_US for p in pulses):
            raise ValueError('Se requieren tres pulsos entre 500 y 2500 us.')

    def _apply(self, pulses):
        values = tuple(int(math.floor(p + 0.5)) for p in pulses)
        for i, (fd, value) in enumerate(zip(self.fds, values)):
            if self.current is None or value != self.current[i]:
                os.lseek(fd, 0, os.SEEK_SET)
                os.write(fd, str(value * 1000).encode('ascii'))
        self.current = values

    def write(self, pulses):
        self._validate(pulses)
        with self.lock:
            if self.error is not None:
                raise RuntimeError('Fallo al actualizar el PWM.') from self.error
            if self.stop.is_set():
                raise RuntimeError('La salida PWM esta cerrada.')
            self._apply(pulses)
            self.last_update = time.monotonic()

    def _watchdog(self):
        while not self.stop.wait(0.02):
            with self.lock:
                if time.monotonic() - self.last_update > self.timeout_s and not self.tripped.is_set():
                    try:
                        self._apply(self.neutral)
                        self.tripped.set()
                    except OSError as error:
                        self.error = error
                        self.stop.set()
                        for path in self.paths:
                            try:
                                (path / 'duty_cycle').write_text('0')
                                (path / 'enable').write_text('0')
                            except OSError:
                                pass

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        for path in self.paths:
            try:
                (path / 'duty_cycle').write_text('0')
                (path / 'enable').write_text('0')
            except OSError:
                pass
        for fd in self.fds:
            os.close(fd)
        self.fds.clear()
        for path in self.paths:
            try:
                (self.chip / 'unexport').write_text(path.name[3:])
            except OSError:
                pass
        self.paths.clear()
        if hasattr(self, 'lock_file') and not self.lock_file.closed:
            self.lock_file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


class RobotController:
    def __init__(self, model=None, lp=6.5, l1=7, l2=4.5, lb=4.6,
                 start_angles=(30, 30, 30), auto_init=True):
        self.robot = model
        self.current_angles = list(validate_angles(start_angles))
        self.pwm = ServoPWM(initial_us=self._calc_pulses(*start_angles))
        try:
            if auto_init:
                self.initialize()
        except BaseException:
            self.shutdown()
            raise

    def _calc_pulses(self, theta1, theta2, theta3):
        angles = validate_angles((theta1, theta2, theta3))
        return tuple(
            angle_to_pulse_us(90.0 - angle + offset)
            for angle, offset in zip(angles, OFFSETS)
        )

    def set_motor_angles(self, theta1, theta2, theta3):
        angles = validate_angles((theta1, theta2, theta3))
        self.pwm.write(self._calc_pulses(*angles))
        self.current_angles = list(angles)

    def initialize(self):
        print("Inicializando...")
        self.set_motor_angles(30, 30, 30)
        self.interpolate_time((20, 20, 20), duration=0.25)
        time.sleep(1)
        self.Goto_time_spherical(0, 0, 9, t=0.25)
        time.sleep(1)
        print("Inicializacion completa.")

    def interpolate_time(self, target_angles, steps=100, duration=0.3,
                         individual_durations=None):
        target_angles = validate_angles(target_angles)
        durations = tuple(
            individual_durations if individual_durations is not None
            else (duration,) * 3
        )
        if len(durations) != 3 or any(
            not math.isfinite(d) or d < 0 for d in durations
        ):
            raise ValueError("Las duraciones deben ser finitas y no negativas.")

        current = tuple(self.current_angles)
        max_duration = max(durations)
        if max_duration == 0:
            self.set_motor_angles(*target_angles)
            return

        # Conserva el intervalo de actualizacion de 10 ms del codigo original.
        steps = max(1, math.ceil(max_duration / 0.01))
        start = time.monotonic()
        for i in range(1, steps + 1):
            elapsed = i * max_duration / steps
            time.sleep(max(0, start + elapsed - time.monotonic()))
            angles = [
                c + (target - c) * min(elapsed / d, 1) if d > 0 else target
                for c, target, d in zip(current, target_angles, durations)
            ]
            self.set_motor_angles(*angles)

    def interpolate_speed(self, target_angles, speed=30, individual_speeds=None):
        target_angles = validate_angles(target_angles)
        speeds = tuple(
            individual_speeds if individual_speeds is not None else (speed,) * 3
        )
        if len(speeds) != 3 or any(
            not math.isfinite(s) or s <= 0 for s in speeds
        ):
            raise ValueError("Las velocidades deben ser positivas y finitas.")
        durations = [
            abs(target - current) / s
            for current, target, s in zip(self.current_angles, target_angles, speeds)
        ]
        self.interpolate_time(target_angles, individual_durations=durations)

    def _kinematic_angles(self, mode, *args):
        if self.robot is None:
            raise RuntimeError("Esta operacion requiere un modelo RobotKinematics.")
        if mode == "spherical":
            self.robot.solve_inverse_kinematics_spherical(*args)
        else:
            self.robot.solve_inverse_kinematics_vector(*args)
        return [
            math.degrees(angle)
            for angle in (self.robot.theta1, self.robot.theta2, self.robot.theta3)
        ]

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        angles = self._kinematic_angles("spherical", theta, phi, h)
        self.interpolate_time(angles, duration=t)

    def Goto_time_vector(self, a, b, c, h, t=0.5):
        angles = self._kinematic_angles("vector", a, b, c, h)
        self.interpolate_time(angles, duration=t)

    def Goto_N_time_vector(self, a, b, c, h):
        self.set_motor_angles(*self._kinematic_angles("vector", a, b, c, h))

    def Goto_N_time_spherical(self, theta, phi, h):
        self.set_motor_angles(*self._kinematic_angles("spherical", theta, phi, h))

    def Dance1(self):
        self.Goto_time_vector(0.258819045103, 0, 0.965925826289, 8)
        for _ in range(3):
            for i in range(100):
                t = 2 * math.pi * i / 100
                x = math.cos(math.pi * 5 / 12) * math.cos(t)
                y = math.cos(math.pi * 5 / 12) * math.sin(t)
                z = math.sin(math.pi * 5 / 12)
                self.Goto_N_time_vector(x, y, z, 8)
                time.sleep(0.01)
        self.Goto_time_vector(0, 0, 1, 8)

    def shutdown(self):
        self.pwm.close()


def main():
    parser = argparse.ArgumentParser(description="Control de tres servos con signo invertido.")
    parser.add_argument("angles", type=float, nargs="*", help="Tres angulos de -90 a 90.")
    parser.add_argument("--hold", type=float, default=2.0, help="Tiempo de prueba en segundos.")
    parser.add_argument("--interactive", action="store_true", help="Recibir angulos por terminal.")
    args = parser.parse_args()

    if len(args.angles) not in (0, 3):
        parser.error("Debes ingresar exactamente tres angulos.")
    if not math.isfinite(args.hold) or args.hold < 0:
        parser.error("El tiempo debe ser finito y no negativo.")
    if args.angles:
        try:
            validate_angles(args.angles)
        except ValueError as error:
            parser.error(str(error))

    controller = None
    try:
        if args.interactive or args.angles:
            initial = args.angles if args.angles else (0, 0, 0)
            controller = RobotController(start_angles=initial, auto_init=False)
            if args.interactive:
                print("Escribe tres angulos, por ejemplo: -30 0 45. Salir: q o Ctrl+C.")
                while True:
                    line = input("> ").strip()
                    if line.lower() in ("q", "salir", "exit"):
                        break
                    if not line:
                        continue
                    try:
                        angles = validate_angles(line.split())
                        controller.set_motor_angles(*angles)
                        print("Angulos invertidos antes del offset:", tuple(-a for a in angles))
                    except ValueError as error:
                        print(error)
            else:
                print("Angulos de entrada:", tuple(initial))
                time.sleep(args.hold)
        else:
            from robotKinematics import RobotKinematics
            controller = RobotController(RobotKinematics(), auto_init=True)
            time.sleep(0.5)
    except (KeyboardInterrupt, EOFError):
        print("\nControl finalizado.")
    finally:
        if controller is not None:
            controller.shutdown()


if __name__ == "__main__":
    main()
