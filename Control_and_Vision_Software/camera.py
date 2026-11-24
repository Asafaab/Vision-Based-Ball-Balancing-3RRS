#!/usr/bin/env python3
"""OV9281 RAW8 1280x800: seguimiento de pelota a 120 fps objetivo.

Uso directo: python3 camera.py [--no-stream] [--fps 30]
Uso como modulo: from camera import CameraModule, CENTER_X, CENTER_Y
Dependencias en Raspberry Pi: numpy, opencv-python, picamera2; simplejpeg
solo si STREAM=True. --fps controla MJPEG; la captura permanece a 120 fps.

BallTracker.update(gray, timestamp_ns) devuelve BallResult. Coordenadas y
radios siempre en pixeles de la imagen original. El centro geometrico del
circulo estima el centroide de la silueta completa, incluso con oclusion.
La prediccion sin observacion se marca valid=False y caduca en 0.12 s.
"""

import argparse
from collections import deque
from dataclasses import dataclass
from types import SimpleNamespace
import math
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

STREAM = True
WIDTH, HEIGHT = 1280, 800
CENTER_X, CENTER_Y = WIDTH / 2.0, HEIGHT / 2.0
CAPTURE_FPS = 120
EXPOSURE_US = 5000
# 5000
#5
#4pm 1.5 y 5000

ANALOGUE_GAIN = 1.5

# Radios en la imagen original; valores iniciales estimados de tu captura.
MIN_RADIUS = 80.0
MAX_RADIUS = 185.0

SEARCH_SCALE = 0.5
ACQUIRE_INTERVAL = 0.10
MAX_PREDICTION_S = 0.12
MIN_EDGE_CONTRAST = 3.0
HOUGH_CANNY = 40.0
HOUGH_VOTES = 24.0


@dataclass(frozen=True)
class BallResult:
    timestamp_ns: int
    state: str
    measured: bool
    valid: bool
    x: float | None = None
    y: float | None = None
    dx: float | None = None
    dy: float | None = None
    distance: float | None = None
    radius: float | None = None
    measured_x: float | None = None
    measured_y: float | None = None
    support: float = 0.0
    age_s: float | None = None


def raw_to_gray(buffer, config):
    width, height = config["size"]
    stride, fmt = config["stride"], config["format"]
    data = np.asarray(buffer, dtype=np.uint8).reshape(-1)
    required = height * stride
    if data.size < required:
        raise RuntimeError("Buffer RAW menor que el tamano configurado")
    if fmt == "R8":
        if stride < width:
            raise RuntimeError("Stride R8 invalido")
        gray = data[:required].reshape(height, stride)[:, :width]
    elif fmt == "R16":
        if stride % 2 or stride < width * 2:
            raise RuntimeError("Stride R16 invalido")
        pixels = data[:required].view("<u2").reshape(height, stride // 2)
        gray = (pixels[:, :width] >> 8).astype(np.uint8)
    else:
        raise RuntimeError(f"Formato RAW no compatible: {fmt}")
    return gray.copy(order="C")


class BallTracker:
    def __init__(self, min_radius=MIN_RADIUS, max_radius=MAX_RADIUS):
        self.min_radius, self.max_radius = min_radius, max_radius
        self.kf = None
        self.radius = None
        self.last_stamp = None
        self.last_seen = None
        self.last_acquire = -math.inf
        self.pending = None
        self.rng = np.random.default_rng(8)
        angles = np.linspace(0, 2 * np.pi, 96, endpoint=False)
        self.directions = np.column_stack((np.cos(angles), np.sin(angles)))

    def _initialize(self, x, y):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.statePost = np.array([[x], [y], [0], [0]], np.float32)
        self.kf.errorCovPost = np.diag([9, 9, 250000, 250000]).astype(np.float32)

    def _predict(self, dt):
        self.kf.transitionMatrix = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            np.float32)
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]],
                     np.float32)
        self.kf.processNoiseCov = g @ g.T * 3500.0**2
        return self.kf.predict().reshape(-1)

    def _robust_circle(self, points, seed, tolerance):
        if len(points) < 28:
            return None
        triplets = points[self.rng.integers(0, len(points), (80, 3))]
        a, b, c = triplets[:, 0], triplets[:, 1], triplets[:, 2]
        u, v = b - a, c - a
        det = 2 * (u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0])
        good = np.abs(det) > 10
        a, u, v, det = a[good], u[good], v[good], det[good]
        if not len(a):
            return None
        uu, vv = (u * u).sum(1), (v * v).sum(1)
        offset = np.column_stack(((uu * v[:, 1] - vv * u[:, 1]) / det,
                                  (u[:, 0] * vv - v[:, 0] * uu) / det))
        centers = a + offset
        radii = np.linalg.norm(offset, axis=1)
        allowed = ((np.abs(radii - seed[2]) < 0.20 * seed[2]) &
                   (np.linalg.norm(centers - seed[:2], axis=1) < tolerance))
        centers, radii = centers[allowed], radii[allowed]
        centers = np.vstack((centers, seed[:2]))
        radii = np.r_[radii, seed[2]]
        if self.radius is not None:
            radii[:] = seed[2]
        residual = np.abs(np.linalg.norm(points[None] - centers[:, None], axis=2)
                          - radii[:, None])
        counts = (residual < 2.0).sum(1)
        index = int(np.argmax(counts))
        keep = residual[index] < 2.0
        if keep.sum() < 28:
            return None
        center, radius = centers[index].copy(), float(radii[index])
        # Ajuste geometrico robusto; con un arco corto se conserva el radio previo.
        fixed_radius = self.radius is not None and keep.sum() < 58
        if fixed_radius:
            radius = float(seed[2])
        for _ in range(3):
            p = points[keep]
            diff = center - p
            dist = np.maximum(np.linalg.norm(diff, axis=1), 1e-6)
            jac = diff / dist[:, None]
            if not fixed_radius:
                jac = np.column_stack((jac, -np.ones(len(p))))
            if np.linalg.cond(jac) > 30:
                return None
            delta = np.linalg.lstsq(jac, radius - dist, rcond=None)[0]
            center += delta[:2]
            if not fixed_radius:
                radius += delta[2]
            keep = np.abs(np.linalg.norm(points - center, axis=1) - radius) < 2.0
            if keep.sum() < 28:
                return None
        angles = np.sort(np.mod(np.arctan2(points[keep, 1] - center[1],
                                           points[keep, 0] - center[0]), 2*np.pi))
        span = 2*np.pi - np.max(np.diff(np.r_[angles, angles[0] + 2*np.pi]))
        if span < np.deg2rad(110):
            return None
        if (abs(radius - seed[2]) > 0.20 * seed[2] or
                np.linalg.norm(center - seed[:2]) > tolerance):
            return None
        return float(center[0]), float(center[1]), float(radius), keep.sum() / 96.0

    def _refine(self, gray, seed, margin):
        x, y, radius = seed
        pad = radius + margin + 10
        h, w = gray.shape
        x0, y0 = max(0, int(x-pad)), max(0, int(y-pad))
        x1, y1 = min(w, int(x+pad+1)), min(h, int(y+pad+1))
        if x1-x0 < 20 or y1-y0 < 20:
            return None
        s = SEARCH_SCALE
        roi = cv2.resize(gray[y0:y1, x0:x1], None, fx=s, fy=s,
                         interpolation=cv2.INTER_AREA)
        roi = cv2.GaussianBlur(roi, (5, 5), 1.0)
        local = np.array([(x-x0)*s, (y-y0)*s, radius*s])
        radii = np.arange(max(2, (radius-margin)*s), (radius+margin)*s+1, 1.0)
        mx = (local[0] + self.directions[:, 0, None] * radii).astype(np.float32)
        my = (local[1] + self.directions[:, 1, None] * radii).astype(np.float32)
        profile = cv2.remap(roi, mx, my, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE).astype(np.float32)
        contrast = np.abs(profile[:, 4:] - profile[:, :-4])
        valid = ((mx[:, 2:-2] > 3) & (mx[:, 2:-2] < roi.shape[1]-4) &
                 (my[:, 2:-2] > 3) & (my[:, 2:-2] < roi.shape[0]-4))
        contrast[~valid] = 0
        # Favorecer el borde cercano al radio previo limita saltos al marco.
        cost = contrast / (1 + 0.018 * np.abs(radii[2:-2] - local[2]))
        index = cost.argmax(1)
        strength = contrast[np.arange(96), index]
        good = strength >= MIN_EDGE_CONTRAST
        rr = radii[index + 2]
        points = local[:2] + self.directions[good] * rr[good, None]
        fitted = self._robust_circle(points, local, margin*s)
        if fitted is None:
            return None
        cx, cy, r, support = fitted
        r /= s
        if not self.min_radius <= r <= self.max_radius:
            return None
        return cx/s+x0, cy/s+y0, r, support

    def _acquire(self, gray):
        s = SEARCH_SCALE
        reduced = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        reduced = cv2.GaussianBlur(reduced, (5, 5), 1.0)
        circles = cv2.HoughCircles(
            reduced, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=self.min_radius*s, param1=HOUGH_CANNY,
            param2=HOUGH_VOTES, minRadius=round(self.min_radius*s),
            maxRadius=round(self.max_radius*s))
        if circles is None:
            return None
        best = None
        for circle in circles[0, :8]:
            candidate = self._refine(gray, circle.astype(float)/s, 28.0)
            if candidate is not None and candidate[3] >= 0.52:
                if best is None or candidate[3] > best[3]:
                    best = candidate
        return best

    def update(self, gray, timestamp_ns):
        if gray.shape != (HEIGHT, WIDTH) or gray.dtype != np.uint8:
            raise ValueError(f"Se requiere imagen gris uint8 {WIDTH}x{HEIGHT}")
        t = timestamp_ns * 1e-9
        if self.last_stamp is not None and timestamp_ns <= self.last_stamp:
            raise ValueError("SensorTimestamp debe ser estrictamente creciente")
        dt = (timestamp_ns-self.last_stamp)*1e-9 if self.last_stamp else 1/CAPTURE_FPS
        self.last_stamp = timestamp_ns
        if self.last_seen is not None and t-self.last_seen > MAX_PREDICTION_S:
            self.kf, self.radius = None, None
        measurement = None
        if self.kf is not None:
            predicted = self._predict(dt)
            age = t-self.last_seen
            margin = min(75.0, 26.0 + age*350.0)
            measurement = self._refine(gray, (*predicted[:2], self.radius), margin)
        elif t-self.last_acquire >= ACQUIRE_INTERVAL:
            self.last_acquire = t
            candidate = self._acquire(gray)
            # Dos adquisiciones coherentes antes de fijar el objetivo.
            if candidate is not None:
                if self.pending is not None:
                    previous, previous_t = self.pending
                    coherent = (t-previous_t < 0.4 and
                                math.hypot(candidate[0]-previous[0],
                                           candidate[1]-previous[1]) < 120 and
                                abs(candidate[2]-previous[2]) < 25)
                    if coherent:
                        measurement = candidate
                        self._initialize(candidate[0], candidate[1])
                        self._predict(dt)
                self.pending = (candidate, t)
            else:
                self.pending = None
        if measurement is not None:
            mx, my, radius, support = measurement
            partial = support < 0.65
            sigma = 4.5 if partial else 2.5
            self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32)*sigma**2
            state = self.kf.correct(np.array([[mx], [my]], np.float32)).reshape(-1)
            if self.radius is None:
                self.radius = radius
            elif not partial:
                self.radius = 0.85*self.radius + 0.15*radius
            self.last_seen = t
            self.pending = None
            status = "ARCO" if partial else "MEDIDO"
        elif self.kf is not None:
            state = self.kf.statePost.reshape(-1)
            status, mx, my, support = "PREDICCION", None, None, 0.0
        else:
            return BallResult(timestamp_ns, "BUSCANDO", False, False)
        x, y = float(state[0]), float(state[1])
        dx, dy = x-CENTER_X, y-CENTER_Y
        return BallResult(timestamp_ns, status, measurement is not None,
                          measurement is not None, x, y, dx, dy, math.hypot(dx, dy),
                          self.radius, mx, my, float(support), t-self.last_seen)


def put_latest(q, item):
    dropped = 0
    try:
        q.get_nowait()
        dropped = 1
    except queue.Empty:
        pass
    q.put_nowait(item)
    return dropped


class Capture:
    def __init__(self):
        self.frames = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.count = self.dropped = self.timestamp = 0
        self.metadata = {}
        self.error = None

    def callback(self, request):
        try:
            metadata = request.get_metadata()
            stamp = int(metadata["SensorTimestamp"])
            gray = raw_to_gray(request.make_buffer("raw"), request.config["raw"])
            dropped = put_latest(self.frames, (gray, stamp, time.monotonic()))
            with self.lock:
                self.count += 1
                self.dropped += dropped
                self.timestamp, self.metadata = stamp, metadata
        except Exception as exc:
            self.error = exc

    def stats(self):
        with self.lock:
            return self.count, self.dropped, self.timestamp, self.metadata.copy()


def draw_result(gray, result, metrics):
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    origin = (round(CENTER_X), round(CENTER_Y))
    cv2.drawMarker(image, origin, (255, 200, 0), cv2.MARKER_CROSS, 25, 2)
    if result.x is not None:
        center = (round(result.x), round(result.y))
        color = (0, 220, 0) if result.measured else (0, 180, 255)
        cv2.circle(image, center, round(result.radius), color, 2, cv2.LINE_AA)
        cv2.circle(image, center, 4, color, -1)
        cv2.line(image, origin, center, color, 1, cv2.LINE_AA)
        info = (f"{result.state}  x={result.x:.1f} y={result.y:.1f}  "
                f"dx={result.dx:+.1f} dy={result.dy:+.1f}  d={result.distance:.1f}px")
    else:
        info = "BUSCANDO PELOTA"
    for y, text in [(30, info), (59, metrics)]:
        cv2.putText(image, text, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return image


class Stream:
    def __init__(self, quality):
        import simplejpeg
        self.encoder = simplejpeg.encode_jpeg
        self.quality = quality
        self.frames = queue.Queue(maxsize=1)
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.closed = False
        self.error = None
        self.worker = threading.Thread(target=self._encode, daemon=True)
        self.worker.start()

    def _encode(self):
        try:
            while not self.closed:
                try:
                    gray, result, metrics = self.frames.get(timeout=0.2)
                except queue.Empty:
                    continue
                image = draw_result(gray, result, metrics)
                jpeg = self.encoder(image, quality=self.quality, colorspace="BGR")
                with self.condition:
                    self.jpeg = jpeg
                    self.sequence += 1
                    self.condition.notify_all()
        except Exception as exc:
            self.error = exc

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.worker.join(timeout=2)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg"):
            self.send_error(404)
            return
        stream = self.server.stream
        self.connection.settimeout(5)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Connection", "close")
        self.end_headers()
        sequence = 0
        try:
            while True:
                with stream.condition:
                    stream.condition.wait_for(
                        lambda: stream.closed or stream.sequence != sequence)
                    if stream.closed:
                        return
                    jpeg, sequence = stream.jpeg, stream.sequence
                self.wfile.write(
                    b"--FRAME\r\nContent-Type: image/jpeg\r\n" +
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
        except (OSError, ConnectionError):
            pass

    def log_message(self, *args):
        pass



@dataclass(frozen=True)
class CameraData:
    """Instantanea de un frame procesado; nunca se repite en read().

    position: pixeles de imagen, X derecha / Y abajo (filtrados por Kalman).
    displacement y velocity: X derecha / Y arriba; velocity en px/s por
    minimos cuadrados sobre las ultimas mediciones validas (hasta 8).
    fps: tasa real de procesamiento/publicacion, ventana de 0.5 segundos.
    capture_fps: tasa de recepcion RAW medida con SensorTimestamp.
    timestamp: tiempo de sensor en segundos, no fecha/hora Unix.
    Con detected=False, position/displacement/velocity/speed son None.
    Las predicciones sin medida solo se usan internamente y en el video.
    """
    detected: bool
    position: tuple[float, float] | None
    center: tuple[float, float]
    displacement: tuple[float, float] | None
    velocity: tuple[float, float] | None
    speed: float | None
    fps: float
    capture_fps: float = 0.0
    timestamp: float = 0.0
    timestamp_ns: int = 0
    dt: float = 0.0
    frame_id: int = 0
    state: str = "BUSCANDO"
    radius: float | None = None
    distance: float | None = None


class _DataAdapter:
    def __init__(self, velocity_samples=8):
        self.positions = deque(maxlen=velocity_samples)
        self.process_times = deque(maxlen=240)
        self.capture_stats = deque(maxlen=240)
        self.previous_stamp = None
        self.frame_id = 0

    def convert(self, result, now, capture_count, capture_stamp):
        stamp = result.timestamp_ns
        dt = ((stamp-self.previous_stamp)*1e-9
              if self.previous_stamp is not None else 0.0)
        self.previous_stamp = stamp
        self.frame_id += 1
        self.process_times.append(now)
        while len(self.process_times) > 2 and now-self.process_times[0] > 0.5:
            self.process_times.popleft()
        elapsed = now-self.process_times[0]
        fps = (len(self.process_times)-1)/elapsed if elapsed > 0 else 0.0
        if not self.capture_stats or capture_stamp > self.capture_stats[-1][1]:
            self.capture_stats.append((capture_count, capture_stamp))
        while (len(self.capture_stats) > 2 and
               capture_stamp-self.capture_stats[0][1] > 500_000_000):
            self.capture_stats.popleft()
        first_count, first_stamp = self.capture_stats[0]
        capture_fps = ((capture_count-first_count)*1e9/(capture_stamp-first_stamp)
                       if capture_stamp > first_stamp else 0.0)
        position = displacement = velocity = speed = distance = None
        detected = bool(result.measured)
        if detected:
            position = (float(result.x), float(result.y))
            displacement = (position[0]-CENTER_X, CENTER_Y-position[1])
            distance = math.hypot(*displacement)
            if self.positions and stamp-self.positions[-1][0] > 100_000_000:
                self.positions.clear()
            self.positions.append((stamp, *displacement))
            if len(self.positions) >= 3:
                # Restar timestamps enteros antes de convertirlos evita perder precision.
                times = np.array([(p[0]-stamp)*1e-9 for p in self.positions])
                times -= times.mean()
                values = np.array([p[1:] for p in self.positions], dtype=float)
                values -= values.mean(axis=0)
                denominator = float(times @ times)
                if denominator > 1e-12:
                    slope = times @ values / denominator
                    velocity = (float(slope[0]), float(slope[1]))
                    speed = math.hypot(*velocity)
        else:
            self.positions.clear()
        return CameraData(
            detected=detected, position=position, center=(CENTER_X, CENTER_Y),
            displacement=displacement, velocity=velocity, speed=speed, fps=float(fps),
            capture_fps=float(capture_fps), timestamp=stamp*1e-9, timestamp_ns=stamp,
            dt=dt, frame_id=self.frame_id, state=result.state,
            radius=result.radius if detected else None, distance=distance)


class CameraModule:
    """Camara asincrona compatible con `with CameraModule() as camera`.

    read(timeout=1.0) espera un resultado nuevo; lanza TimeoutError si no llega.
    Si el main es mas lento que la vision, entrega el ultimo resultado y
    descarta los intermedios para no acumular retraso. Un unico consumidor.
    Importar el modulo o construir la instancia no abre la camara.
    """
    def __init__(self, camera=0, port=8890, stream=None, fps=30,
                 exposure=EXPOSURE_US, gain=ANALOGUE_GAIN, quality=80,
                 min_radius=MIN_RADIUS, max_radius=MAX_RADIUS, threads=1,
                 velocity_samples=8, verbose=False):
        duration = round(1_000_000/CAPTURE_FPS)
        if not 0 < exposure < duration:
            raise ValueError(f"exposure debe estar entre 1 y {duration-1} us")
        if not math.isfinite(gain) or gain < 1 or not 1 <= quality <= 100:
            raise ValueError("quality: 1..100; gain: finito y al menos 1")
        if not 1 <= port <= 65535 or not 1 <= threads <= 8:
            raise ValueError("port: 1..65535; threads: 1..8")
        if not 10 <= min_radius < max_radius <= min(WIDTH, HEIGHT)/2:
            raise ValueError("Radios: 10 <= min_radius < max_radius <= 400")
        if fps not in (15, 30, 60):
            raise ValueError("fps de MJPEG debe ser 15, 30 o 60")
        if not isinstance(velocity_samples, int) or not 3 <= velocity_samples <= 120:
            raise ValueError("velocity_samples debe ser un entero entre 3 y 120")
        self.args = SimpleNamespace(
            camera=camera, port=port, stream=STREAM if stream is None else bool(stream),
            fps=fps, exposure=exposure, gain=gain, quality=quality,
            min_radius=min_radius, max_radius=max_radius, threads=threads,
            verbose=verbose)
        self._adapter = _DataAdapter(velocity_samples)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._error = None
        self._closed = False
        self._latest = None
        self._sequence = self._read_sequence = 0

    def start(self):
        if self._closed:
            raise RuntimeError("CameraModule cerrado; crea una instancia nueva")
        if self._thread is None:
            self._thread = threading.Thread(target=self._worker,
                                            name="camera-processing", daemon=True)
            self._thread.start()
        try:
            if not self._ready.wait(timeout=15.0):
                raise TimeoutError("La camara no termino de iniciar en 15 s")
            if self._error is not None:
                raise RuntimeError("No se pudo iniciar la camara") from self._error
            return self
        except BaseException:
            self.close()
            raise

    def _worker(self):
        try:
            _run_camera(self.args, self)
        except Exception as exc:
            with self._condition:
                self._error = exc
        finally:
            with self._condition:
                self._closed = True
                self._ready.set()
                self._condition.notify_all()

    def _publish(self, result, now, count, stamp):
        data = self._adapter.convert(result, now, count, stamp)
        with self._condition:
            self._latest = data
            self._sequence += 1
            self._condition.notify_all()

    def read(self, timeout=1.0):
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            raise ValueError("timeout debe ser None o un numero finito no negativo")
        if self._thread is None:
            raise RuntimeError("Usa with CameraModule() o camera.start() antes de read()")
        with self._condition:
            ready = self._condition.wait_for(
                lambda: self._error is not None or self._closed or self._stop.is_set()
                or self._sequence != self._read_sequence, timeout=timeout)
            if self._error is not None:
                raise RuntimeError("Error en el modulo de camara") from self._error
            if self._closed or self._stop.is_set():
                raise RuntimeError("CameraModule cerrado")
            if not ready:
                raise TimeoutError("No hay un frame nuevo dentro del tiempo indicado")
            self._read_sequence = self._sequence
            return self._latest

    def close(self):
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=10.0)
            if self._thread.is_alive():
                raise RuntimeError("El hilo de camara no termino de cerrar en 10 s")
        self._closed = True

    stop = close

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False


def _run_camera(args, module):
    duration = round(1_000_000/CAPTURE_FPS)
    cv2.setNumThreads(args.threads)
    from picamera2 import Picamera2
    capture = Capture()
    tracker = BallTracker(args.min_radius, args.max_radius)
    camera = Picamera2(args.camera)
    stream = server = server_thread = None
    started = False
    try:
        modes = camera.sensor_modes
        candidates = [m for m in modes if tuple(m["size"]) == (WIDTH, HEIGHT)
                      and m["bit_depth"] == 8]
        if not candidates:
            raise RuntimeError(f"No existe el modo 1280x800 RAW8. Modos: {modes}")
        mode = max(candidates, key=lambda m: m["fps"])
        if mode["fps"] < CAPTURE_FPS:
            raise RuntimeError(f"El driver anuncia solo {mode['fps']:.2f} fps RAW8")
        controls = {"FrameDurationLimits": (duration, duration),
                    "ExposureTime": args.exposure, "AnalogueGain": args.gain}
        if "AeEnable" in camera.camera_controls:
            controls["AeEnable"] = False
        config = camera.create_video_configuration(
            main={"size": (320, 200), "format": "YUV420"},
            raw={"size": (WIDTH, HEIGHT), "format": "R8"},
            sensor={"output_size": (WIDTH, HEIGHT), "bit_depth": 8},
            controls=controls, buffer_count=8, queue=False)
        camera.configure(config)
        actual = camera.camera_configuration()
        if (actual["raw"]["format"] not in ("R8", "R16") or
                tuple(actual["raw"]["size"]) != (WIDTH, HEIGHT) or
                actual["sensor"]["bit_depth"] != 8 or
                tuple(actual["sensor"]["output_size"]) != (WIDTH, HEIGHT)):
            raise RuntimeError(f"Configuracion incompatible con RAW8: {actual}")
        if args.stream:
            stream = Stream(args.quality)
            server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
            server.daemon_threads = True
            server.stream = stream
            server_thread = threading.Thread(target=server.serve_forever, daemon=True)
            server_thread.start()
        camera.post_callback = capture.callback
        camera.start(show_preview=False)
        started = True
        module._ready.set()
        if args.verbose:
            print(f"Sensor: {WIDTH}x{HEIGHT} RAW8; objetivo {CAPTURE_FPS} fps")
            print(f"Exposicion solicitada: {args.exposure} us; ganancia: {args.gain}")
            print(f"Maximo del driver: {mode['fps']:.2f} fps; RAW: {actual['raw']['format']}")
            print(f"Centro de imagen: ({CENTER_X:.0f}, {CENTER_Y:.0f}); dx derecha+, dy abajo+")
            if stream:
                print(f"MJPEG {args.fps} fps: http://IP_DE_LA_RASPBERRY:{args.port}/stream.mjpg")
            else:
                print("STREAM=False: sin servidor HTTP ni codificacion JPEG")
            print("Detener: Ctrl+C", flush=True)
        last_report = last_frame = time.monotonic()
        next_stream = 0.0
        previous_count = previous_stamp = previous_dropped = previous_encoded = 0
        processed = measured_count = 0
        times_ms, ages_ms = [], []
        metrics = "Objetivo 120 fps; esperando medicion..."
        while not module._stop.is_set():
            if capture.error:
                raise RuntimeError("Error al capturar RAW8") from capture.error
            if stream and stream.error:
                raise RuntimeError("Error al codificar MJPEG") from stream.error
            try:
                gray, stamp, arrival = capture.frames.get(timeout=0.1)
            except queue.Empty:
                if time.monotonic()-last_frame > 5:
                    raise RuntimeError("La camara no entrega imagenes desde hace 5 s")
                continue
            last_frame = time.monotonic()
            begin = time.perf_counter()
            result = tracker.update(gray, stamp)
            times_ms.append((time.perf_counter()-begin)*1000)
            ages_ms.append((time.monotonic()-arrival)*1000)
            processed += 1
            measured_count += int(result.measured)
            now = time.monotonic()
            count_now, _, stamp_now, _ = capture.stats()
            module._publish(result, now, count_now, stamp_now)
            if stream and now >= next_stream:
                put_latest(stream.frames, (gray, result, metrics))
                if next_stream == 0:
                    next_stream = now
                next_stream += (int((now-next_stream)*args.fps)+1)/args.fps
            elapsed = now-last_report
            if elapsed >= 2:
                count, dropped, sensor_stamp, metadata = capture.stats()
                received = ((count-previous_count)*1e9/(sensor_stamp-previous_stamp)
                            if previous_stamp and sensor_stamp > previous_stamp else
                            count/elapsed)
                encoded = stream.sequence if stream else 0
                metrics = f"Captura {received:.1f} fps | Proceso {processed/elapsed:.1f} fps"
                if args.verbose:
                    print(
                        f"{metrics} | Mediciones {measured_count/elapsed:.1f}/s | "
                        f"MJPEG {(encoded-previous_encoded)/elapsed:.1f} fps | "
                        f"Descartes {dropped-previous_dropped} | "
                        f"Proceso p95 {np.percentile(times_ms, 95):.2f} ms | "
                        f"Callback-salida p95 {np.percentile(ages_ms, 95):.2f} ms | "
                        f"Exposure {metadata.get('ExposureTime', '?')} us | "
                        f"Gain {metadata.get('AnalogueGain', '?')} | "
                        f"FrameDuration {metadata.get('FrameDuration', '?')} us",
                        flush=True)
                    if result.x is not None:
                        print(f"{result.state}: x={result.x:.2f}, y={result.y:.2f}, "
                              f"dx={result.dx:+.2f}, dy={result.dy:+.2f}, "
                              f"distancia={result.distance:.2f} px, "
                              f"radio={result.radius:.1f}, soporte={result.support:.0%}, "
                              f"sin observacion={result.age_s*1000:.1f} ms", flush=True)
                    else:
                        print(result.state, flush=True)
                previous_count, previous_stamp = count, sensor_stamp
                previous_dropped, previous_encoded = dropped, encoded
                processed = measured_count = 0
                times_ms.clear()
                ages_ms.clear()
                last_report = now
    finally:
        cleanup_error = None
        actions = []
        if started:
            actions.append(camera.stop)
        actions.append(camera.close)
        if stream:
            actions.append(stream.close)
        if server_thread:
            actions.extend([server.shutdown, lambda: server_thread.join(timeout=2)])
        if server:
            actions.append(server.server_close)
        for action in actions:
            try:
                action()
            except Exception as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if cleanup_error is not None:
            raise cleanup_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=30,
                        help="FPS de MJPEG; captura y tracking tienen objetivo 120")
    parser.add_argument("--exposure", type=int, default=EXPOSURE_US)
    parser.add_argument("--gain", type=float, default=ANALOGUE_GAIN)
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--min-radius", type=float, default=MIN_RADIUS)
    parser.add_argument("--max-radius", type=float, default=MAX_RADIUS)
    parser.add_argument("--threads", type=int, default=1)
    switches = parser.add_mutually_exclusive_group()
    switches.add_argument("--stream", dest="stream", action="store_true")
    switches.add_argument("--no-stream", dest="stream", action="store_false")
    parser.set_defaults(stream=STREAM)
    args = parser.parse_args()
    try:
        with CameraModule(**vars(args)) as camera:
            while True:
                try:
                    camera.read(timeout=1.0)
                except TimeoutError:
                    continue
    except KeyboardInterrupt:
        print("\nSeguimiento detenido.")


if __name__ == "__main__":
    main()
