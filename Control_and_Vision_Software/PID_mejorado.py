import math
import time

# ---------------------------------------------------------------------------
# Elige la version de ganancias (ajustadas en simulacion calibrada con el log del 19-oct-2026):
#   "v1": mas conservadora, menos rebote (recomendada para empezar)
#   "v2": mas rapida (~2 s), mas sobreimpulso; puede oscilar si la planta es sensible
VERSION = "v1"
# ---------------------------------------------------------------------------
_GANANCIAS = {
    #        KP        KI        KD       ALPHA  PHI_OFFSET (giro de ejes, grados)
    "v1": (0.005861, 0.004828, 0.002889, 0.80,   17.5),
    "v2": (0.006961, 0.007928, 0.002989, 0.80,  17.5),
}
KP, KI, KD, ALPHA, PHI_OFFSET = _GANANCIAS[VERSION]
BETA = 0.3
# PHI_OFFSET: giro entre el eje de la inclinacion mandada y el eje en que se mueve la pelota
# (estimado del log: ~17.5 grados). Si la pelota "se va de lado" al corregir, prueba 0 o entre 10 y 25.


def crear_pid(max_theta=10.0, version=None):
    """Devuelve el PID con las ganancias de `version` ("v1" o "v2"; por defecto VERSION)."""
    kp, ki, kd, alpha, phi = _GANANCIAS[version or VERSION]
    return PIDcontroller(kp, ki, kd, alpha, BETA, max_theta, conversion="tanh", phi_offset=phi)


class PIDcontroller:
    """Reemplazo directo de PID.py: misma interfaz, pid(target, current) -> (theta, phi) en grados.

    Mejoras: dt limitado, reinicio automatico si se pierde la pelota, sin golpe de derivada
    en la primera muestra, y anti-windup (integral limitada).
    """
    def __init__(self, kp, ki, kd, alpha, beta, max_theta, conversion="linear",
                 dt_max=0.05, gap_reset=0.2, i_max_deg=4.0, phi_offset=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.alpha, self.beta, self.max_theta = alpha, beta, max_theta
        self.tanh = (conversion == "tanh")
        self.phi_offset = phi_offset    # grados: giro de la direccion de inclinacion
        self.dt_max = dt_max            # s: tope de dt para integral y derivada
        self.gap_reset = gap_reset      # s: si pasa mas tiempo sin llamar -> se reinicia el estado
        # limite de la integral: aporte maximo i_max_deg grados de inclinacion
        self.i_lim = (i_max_deg / (15.0 * beta)) / ki if ki > 0 else float("inf")
        self.reset()

    def reset(self):
        self.prev_out_x = self.prev_out_y = 0.0
        self.prev_err_x = self.prev_err_y = 0.0
        self.sum_err_x = self.sum_err_y = 0.0
        self.last_time = None

    def pid(self, target, current):
        now = time.perf_counter()
        if self.last_time is not None and now - self.last_time > self.gap_reset:
            self.reset()                # se perdio la pelota: no arrastrar integral ni derivada vieja
        first = self.last_time is None
        dt = 0.01 if first else min(max(now - self.last_time, 1e-4), self.dt_max)

        err_x = current[0] - target[0]
        err_y = current[1] - target[1]
        self.sum_err_x = max(-self.i_lim, min(self.i_lim, self.sum_err_x + err_x * dt))
        self.sum_err_y = max(-self.i_lim, min(self.i_lim, self.sum_err_y + err_y * dt))
        d_x = 0.0 if first else (err_x - self.prev_err_x) / dt
        d_y = 0.0 if first else (err_y - self.prev_err_y) / dt

        pid_x = self.kp * err_x + self.ki * self.sum_err_x + self.kd * d_x
        pid_y = self.kp * err_y + self.ki * self.sum_err_y + self.kd * d_y
        fx = self.alpha * pid_x + (1 - self.alpha) * self.prev_out_x
        fy = self.alpha * pid_y + (1 - self.alpha) * self.prev_out_y

        phi = (math.degrees(math.atan2(fy, fx)) + self.phi_offset) % 360
        r = math.hypot(fx, fy)
        if self.tanh:
            theta = min(max(0.0, 15 * math.tanh(self.beta * r)), self.max_theta)
        else:
            theta = min(max(0.0, self.beta * r), self.max_theta)

        self.prev_err_x, self.prev_err_y = err_x, err_y
        self.prev_out_x, self.prev_out_y = fx, fy
        self.last_time = now
        return theta, phi
