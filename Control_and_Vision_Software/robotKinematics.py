import argparse
import numpy as np


class KinematicsError(ValueError):
    pass


class RobotKinematics:
    """Longitudes en cm; entradas angulares en grados; theta1/2/3 en radianes.

    theta se mide desde la horizontal radial exterior, como en MATLAB.
    controller.py debe usar math.degrees(theta), sin restarlo de 90.
    model='original' reproduce el modelo seleccionado por defecto en la GUI.
    model='rigid' reproduce la opcion de plataforma rigida de esa GUI.
    Las filas de A/B/C son los puntos 1/2/3 (MATLAB los usa por columnas).
    invert=False selecciona mayor proyeccion radial; True, la rama opuesta.
    No se aplican offsets, limites de servo ni recorte automatico de posturas.
    """

    def __init__(self, lp=6.5, l1=7.0, l2=4.5, lb=4.6,
                 invert=False, model="original"):
        dimensions = np.asarray((lp, l1, l2, lb), dtype=float)
        if not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0):
            raise ValueError("Las cuatro longitudes deben ser positivas y finitas.")
        if model not in ("original", "rigid"):
            raise ValueError("model debe ser 'original' o 'rigid'.")

        self.lp, self.l1, self.l2, self.lb = dimensions.tolist()
        self.invert = bool(invert)
        self.model = model
        psi = np.deg2rad((120.0, 0.0, 240.0))
        self._er = np.column_stack((np.cos(psi), np.sin(psi), np.zeros(3)))
        self._base = self.lb * self._er
        self._local_top = self.lp * self._er
        self._pending = None
        self.maxtheta = None

        # Limites geometricos con plataforma horizontal, sin limites de servos.
        self.minh = self.compute_minh()
        self.maxh = self.compute_maxh()
        h0 = 0.8 * self.maxh
        if h0 <= self.minh:
            h0 = (self.minh + self.maxh) / 2
        self.solve_inverse_kinematics_roll_pitch(0.0, 0.0, h0)

    def compute_maxh(self):
        value = (self.l1 + self.l2) ** 2 - (self.lp - self.lb) ** 2
        if value <= 0:
            raise KinematicsError("No existe una postura horizontal con altura positiva.")
        return float(np.sqrt(value))

    def compute_minh(self):
        value = (self.l1 - self.l2) ** 2 - (self.lp - self.lb) ** 2
        return float(np.sqrt(max(0.0, value)))

    @staticmethod
    def _rotation(roll, pitch):
        if not np.all(np.isfinite((roll, pitch))):
            raise ValueError("Roll y pitch deben ser finitos.")
        roll, pitch = np.deg2rad((roll, pitch))
        sr, cr = np.sin(roll), np.cos(roll)
        sp, cp = np.sin(pitch), np.cos(pitch)
        return np.array(((cp, sp * sr, sp * cr),
                         (0.0, cr, -sr),
                         (-sp, cp * sr, cp * cr)))

    @staticmethod
    def _normal(a, b, c):
        normal = np.asarray((a, b, c), dtype=float)
        if not np.all(np.isfinite(normal)):
            raise ValueError("La normal debe contener valores finitos.")
        scale = np.max(np.abs(normal))
        if scale == 0:
            raise ValueError("La normal no puede ser el vector cero.")
        normal = normal / scale
        return normal / np.linalg.norm(normal)

    def _top(self, normal, h, rotation=None):
        if not np.isfinite(h) or h <= 0:
            raise ValueError("La altura debe ser positiva y finita.")
        center = np.array((0.0, 0.0, float(h)))
        if self.model == "rigid":
            if rotation is None:
                # Recupera Ry(pitch) @ Rx(roll), con roll entre -90 y 90 grados.
                roll = np.rad2deg(np.arcsin(np.clip(-normal[1], -1.0, 1.0)))
                pitch = np.rad2deg(np.arctan2(normal[0], normal[2]))
                rotation = self._rotation(roll, pitch)
            return self._local_top @ rotation.T + center

        d = self._er @ normal
        denominator = np.hypot(normal[2], d)
        if np.any(denominator < 1e-10):
            raise KinematicsError("Orientacion singular para el modelo original.")
        directions = normal[2] * self._er.copy()
        directions[:, 2] -= d
        return center + self.lp * directions / denominator[:, None]

    def _middle(self, top):
        delta = top - self._base
        u = np.sum(delta * self._er, axis=1)
        w = delta[:, 2]
        rho = np.hypot(u, w)
        if np.any(rho < 1e-10):
            raise KinematicsError("Configuracion singular de un brazo.")

        k = (np.sum(delta * delta, axis=1) + self.l2**2 - self.l1**2) / (2 * self.l2)
        q = k / rho
        invalid = np.flatnonzero(np.abs(q) > 1.0 + 1e-10)
        if invalid.size:
            legs = ", ".join(str(i + 1) for i in invalid)
            raise KinematicsError(f"Postura fuera de alcance para los servos: {legs}.")

        opening = np.arccos(np.clip(q, -1.0, 1.0))
        candidates = np.arctan2(w, u)[:, None] + opening[:, None] * np.array((-1.0, 1.0))
        radial = np.cos(candidates)
        selection = np.argmin(radial, axis=1) if self.invert else np.argmax(radial, axis=1)
        angles = candidates[np.arange(3), selection]
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        angles[np.abs(angles + np.pi) < np.deg2rad(1e-10)] = np.pi

        middle = self._base + self.l2 * np.cos(angles)[:, None] * self._er
        middle[:, 2] += self.l2 * np.sin(angles)
        residual = max(
            np.max(np.abs(np.linalg.norm(top - middle, axis=1) - self.l1)),
            np.max(np.abs(np.linalg.norm(middle - self._base, axis=1) - self.l2)),
        )
        if not np.isfinite(residual) or residual > 1e-7 * max(1.0, self.l1, self.l2):
            raise KinematicsError("No se satisface el cierre geometrico.")
        return middle, angles

    def _commit(self, top, middle, angles, normal, h):
        self.A = top.copy()
        self.B = self._base.copy()
        self.C = middle.copy()
        self.A1, self.A2, self.A3 = self.A.copy()
        self.B1, self.B2, self.B3 = self.B.copy()
        self.C1, self.C2, self.C3 = self.C.copy()
        self.theta1, self.theta2, self.theta3 = map(float, angles)
        self.normal = normal.copy()
        self.h = float(h)
        self.p = np.array((0.0, 0.0, self.h))
        self._pending = None
        return angles.copy()

    @property
    def angles_deg(self):
        return np.rad2deg((self.theta1, self.theta2, self.theta3))

    def solve_top(self, a, b, c, h):
        """Prepara los puntos superiores; solve_middle confirma la solucion."""
        normal = self._normal(a, b, c)
        top = self._top(normal, h)
        self._pending = (top, normal, h)
        return top.copy()

    def solve_middle(self):
        if self._pending is None:
            top, normal, h = self.A, self.normal, self.h
        else:
            top, normal, h = self._pending
        middle, angles = self._middle(top)
        self._commit(top, middle, angles, normal, h)
        return middle.copy()

    def solve_inverse_kinematics_vector(self, a, b, c, h):
        normal = self._normal(a, b, c)
        top = self._top(normal, h)
        middle, angles = self._middle(top)
        return self._commit(top, middle, angles, normal, h)

    def solve_inverse_kinematics_spherical(self, theta, phi, h):
        """theta: inclinacion de la normal; phi: azimut. Ambos en grados."""
        if not np.all(np.isfinite((theta, phi))):
            raise ValueError("La inclinacion y el azimut deben ser finitos.")
        theta, phi = np.deg2rad((theta, phi))
        return self.solve_inverse_kinematics_vector(
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta), h,
        )

    def solve_inverse_kinematics_roll_pitch(self, roll, pitch, h):
        """Entrada directa igual a los controles Roll/Pitch de MATLAB."""
        rotation = self._rotation(roll, pitch)
        normal = rotation[:, 2]
        top = self._top(normal, h, rotation)
        middle, angles = self._middle(top)
        return self._commit(top, middle, angles, normal, h)

    def max_theta(self, h, tol=1e-3, phi=0.0, limit=25.0):
        """Estima el primer limite geometrico en UN azimut (no todo el espacio).

        tol esta en radianes. No verifica colisiones ni limites de servos.
        Es una consulta opcional: no se ejecuta en cada actualizacion.
        """
        if not np.all(np.isfinite((h, tol, phi, limit))) or tol <= 0 or not 0 <= limit <= 90:
            raise ValueError("Parametros no validos para max_theta.")
        phi = np.deg2rad(phi)

        def reachable(theta):
            normal = np.array((np.sin(theta) * np.cos(phi),
                               np.sin(theta) * np.sin(phi), np.cos(theta)))
            try:
                self._middle(self._top(normal, h))
                return True
            except KinematicsError:
                return False

        if not reachable(0.0):
            raise KinematicsError("La postura horizontal no es alcanzable a esa altura.")
        lower = 0.0
        for upper in np.deg2rad(np.linspace(0.0, limit, 101)[1:]):
            if not reachable(upper):
                while upper - lower > tol:
                    middle = (lower + upper) / 2
                    if reachable(middle):
                        lower = middle
                    else:
                        upper = middle
                break
            lower = upper
        self.maxtheta = float(np.rad2deg(lower))
        return self.maxtheta


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Cinematica equivalente al resolver de MATLAB.")
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--height", type=float, default=9.0)
    parser.add_argument("--model", choices=("original", "rigid"), default="original")
    args = parser.parse_args()
    robot = RobotKinematics(model=args.model)
    robot.solve_inverse_kinematics_roll_pitch(args.roll, args.pitch, args.height)
    print("Theta MATLAB [grados]:", np.round(robot.angles_deg, 6))
    print("Entrada controller [grados]:", np.round(robot.angles_deg, 6))
