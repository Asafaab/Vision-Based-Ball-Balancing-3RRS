"""Composición de la inclinación del PID con el pitch del chasis."""
import math
import time


PITCH_MAX_DEG = 15.0
PITCH_RATE_DEG_S = 45.0
IMU_MAX_AGE_S = 0.25
MAX_INCLINACION_DEG = 20.0


class CompensacionPitch:
    def __init__(self, azimut_deg=0.0, cero_deg=0.0):
        # Azimut de la normal que produciría un pitch positivo del chasis.
        azimut = math.radians(azimut_deg)
        self.cos_azimut = math.cos(azimut)
        self.sin_azimut = math.sin(azimut)
        self.cero_deg = cero_deg
        self.reset()

    def reset(self):
        self.pitch_deg = 0.0
        self._t_prev = time.perf_counter()

    def actualizar(self, pitch_imu, ahora):
        dt = max(0.0, min(ahora - self._t_prev, 0.1))
        self._t_prev = ahora
        objetivo = 0.0 if pitch_imu is None else pitch_imu - self.cero_deg
        objetivo = max(-PITCH_MAX_DEG, min(PITCH_MAX_DEG, objetivo))
        paso = PITCH_RATE_DEG_S * dt
        self.pitch_deg += max(-paso, min(paso, objetivo - self.pitch_deg))

    def normal(self, theta_deg, phi_deg):
        """Devuelve la normal en el chasis y si se recortó la corrección PID."""
        theta, phi = math.radians(theta_deg), math.radians(phi_deg)
        pitch = math.radians(self.pitch_deg)
        cp, sp = math.cos(pitch), math.sin(pitch)
        ca, sa = self.cos_azimut, self.sin_azimut
        cf, sf = math.cos(phi), math.sin(phi)

        # En una base que apunta a lo largo de la pendiente, aplicar Ry(-pitch).
        longitudinal = cf * ca + sf * sa
        transversal = -cf * sa + sf * ca

        def altura_normal(t):
            return sp * longitudinal * math.sin(t) + cp * math.cos(t)

        min_z = math.cos(math.radians(MAX_INCLINACION_DEG))
        limitada = altura_normal(theta) < min_z
        if limitada:
            # Reducir theta del PID conservando el pitch de compensación y su azimut.
            inferior, superior = 0.0, theta
            for _ in range(12):
                medio = (inferior + superior) / 2.0
                if altura_normal(medio) >= min_z:
                    inferior = medio
                else:
                    superior = medio
            theta = inferior

        x = cp * longitudinal * math.sin(theta) - sp * math.cos(theta)
        y = transversal * math.sin(theta)
        return (ca * x - sa * y, sa * x + ca * y, altura_normal(theta)), limitada
