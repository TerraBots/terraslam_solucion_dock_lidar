"""Docking del Create 3 usando solo el LiDAR.

Entradas permitidas por el reto:  /scan  /tf  /tf_static  /dock_status
Salida:                           /cmd_vel

Arquitectura
------------
  percepcion  (callback de /scan, 10 Hz)
      DockDetector encuentra el eje del dock en base_link a partir de las
      medidas del marcador. La deteccion se lleva a odom con el TF del
      instante del scan y se fusiona en un filtro de Kalman estatico (el dock
      no se mueve). La incertidumbre de cada medida crece con la distancia; la
      del estado crece con lo que se mueve el robot (deriva de odometria).
      Asi las lecturas cercanas, las mas precisas, dominan al final, y si
      se pierde el marcador unos ciclos el robot sigue guiado por odometria.

  control  (timer 20 Hz)
      Todo se expresa en el frame del dock: d = distancia a la pared a lo
      largo de su normal, e = error lateral respecto al eje. Maquina de
      estados:

        SEARCH    sin estimacion: esperar unos scans y girar 90 grados (por
                  si algo del propio robot tapa un sector del LiDAR)
        EXPLORE   sin marcador (ocluido o visto demasiado de canto):
                  avanzar hacia el centro del espacio libre que ve el LiDAR
                  y volver a buscar
        STAGE     fuera del cono de aproximacion: ir a un punto sobre el eje
        APPROACH  seguir el eje (pure pursuit sobre la recta) corrigiendo
                  lateral y angulo a la vez, con velocidad segun d
        DOCK      ultimo tramo a velocidad de rampa hasta is_docked
        WAIT_DOCK parado en d_floor: esperar el estado antes de asentar
        SEAT      asentamiento recto de pocos milimetros, sin retroceso largo
        SWEEP     sin contacto en la distancia de acople: barrido de rumbo en
                  el sitio por pulsos (+-1.5, +-3 ... grados), parando quieto
                  tras cada pulso para leer dock_status. El dock fisico puede
                  estar algo girado respecto a las cajas.
        CONFIRM   parado: exigir varias lecturas is_docked estables
        BACKOFF   si llega desalineado o tras un choque: reintentar
        ESCAPE    el parachoques detecto un choque (/hazard_detection):
                  retroceder recto para despegarse y pasar a BACKOFF (o a
                  SEARCH si aun no hay estimacion)
        DONE      acoplado confirmado: mantener parada y vigilar el contacto
        STOPPED   fallo o reintentos agotados: parada hasta relanzar el nodo

Deteccion del marcador
----------------------
El marcador son dos cajas que sobresalen de una pared, separadas por un hueco.
Lo unico que se asume son sus MEDIDAS (relativas), nunca su posicion:

    ancho de cada caja      BOX_W     8.0 cm
    cuanto sobresalen       BOX_D     8.0 cm
    hueco entre cajas       GAP       9.5 cm   (17.5 cm entre centros)

Pipeline por scan (todo en el frame del robot, base_link):

  1. Lineas candidatas a pared con RANSAC secuencial.
  2. Para cada linea, los puntos que sobresalen ~BOX_D hacia el robot se
     agrupan a lo largo de la linea; se buscan dos grupos del ancho de una caja
     separados BOX_W + GAP, con pared visible en el hueco y nada raro a los
     lados.
  3. Refinado: la pared (local, alrededor de las cajas) y las caras frontales
     se ajustan JUNTAS como dos rectas paralelas que comparten normal. La
     pared aporta longitud (orientacion precisa) y las caras frontales aportan
     puntos cercanos cuando la pared queda lejos.
  4. Triangulacion de los cuatro bordes de las cajas con precision
     sub-muestra: cada borde esta entre el ultimo rayo que toca la caja y el
     primero que no; se toma la bisectriz de esos dos rayos y se intersecta con
     la cara frontal. Si el vecino cae en la cara lateral de la caja, esa cara
     da el borde directamente.
  5. El eje del dock es el promedio de los centros de las dos cajas
     (equivalente al centro del hueco, pero usando los cuatro bordes).
"""


import math
from collections import deque
from time import monotonic
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetection, \
    HazardDetectionVector
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ======================================================================
# Percepcion: deteccion del marcador (numpy puro)
# ======================================================================


BOX_W = 0.080
BOX_D = 0.080
GAP = 0.095
PITCH = BOX_W + GAP            # 0.175 entre centros de caja

# Limites fisicos del laboratorio Kalman, tambien aplicados en cmd().
MAX_LINEAR_SPEED = 0.20
MAX_ANGULAR_SPEED = 0.50
MAX_SEAT_TRAVEL = 0.010        # presupuesto absoluto de asentamiento [m]


@dataclass
class Detection:
    """Eje del dock en el frame de los puntos de entrada.

    wall_pt   punto de la pared (cara interior) sobre el eje del dock
    normal    normal unitaria de la pared, apuntando hacia la sala
    """
    wall_pt: np.ndarray
    normal: np.ndarray
    front_offset: float           # distancia medida pared -> cara de cajas
    box_centers: np.ndarray       # (2, 2) centros de las caras frontales
    edges: np.ndarray             # (4,) coordenada s de los 4 bordes
    n_wall: int
    n_front: int
    residual: float               # rms del ajuste conjunto [m]
    score: float
    debug: dict = field(default_factory=dict)

    @property
    def tangent(self):
        return np.array([-self.normal[1], self.normal[0]])

    def distance_from(self, p):
        """Distancia de p a la pared, medida a lo largo de la normal."""
        return float(np.dot(np.asarray(p) - self.wall_pt, self.normal))

    def lateral_of(self, p):
        return float(np.dot(np.asarray(p) - self.wall_pt, self.tangent))


def scan_to_points(ranges, angle_min, angle_inc, range_min, range_max,
                   R=np.eye(2), t=np.zeros(2), max_range=6.0):
    """Rayos validos del scan transformados al frame destino (R, t).

    Devuelve (pts, ang, idx): puntos (N, 2) en el frame destino, angulo de
    cada rayo en el frame del laser e indice original, para poder razonar
    sobre rayos vecinos.
    """
    r = np.asarray(ranges, dtype=float)
    n = r.size
    ang = angle_min + np.arange(n) * angle_inc
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_range))
    idx = np.nonzero(ok)[0]
    lp = np.stack([r[idx] * np.cos(ang[idx]), r[idx] * np.sin(ang[idx])], 1)
    return lp @ R.T + t, ang[idx], idx


def _fit_line(p):
    """Recta de minimos cuadrados totales: (centroide, normal unitaria)."""
    c = p.mean(0)
    _, _, vt = np.linalg.svd(p - c, full_matrices=False)
    return c, vt[1]


def ransac_lines(pts, rng, max_lines=6, tol=0.012, min_inliers=12,
                 iters=60):
    """RANSAC secuencial. Devuelve [(centroide, normal, mascara_inliers)]."""
    lines = []
    free = np.ones(len(pts), bool)
    for _ in range(max_lines):
        cand = np.nonzero(free)[0]
        if cand.size < min_inliers:
            break
        best, best_n = None, 0
        # Pares de puntos cercanos en el scan (misma pared con mas
        # probabilidad) y alguno lejano para no perder paredes largas.
        a = rng.choice(cand, iters)
        off = rng.integers(3, 40, iters) * rng.choice([-1, 1], iters)
        b_pos = np.clip(np.searchsorted(cand, a) + off, 0, cand.size - 1)
        b = cand[b_pos]
        for i, j in zip(a, b):
            if i == j:
                continue
            d = pts[j] - pts[i]
            L = math.hypot(*d)
            if L < 0.05:
                continue
            nrm = np.array([-d[1], d[0]]) / L
            dist = np.abs((pts[cand] - pts[i]) @ nrm)
            k = int((dist < tol).sum())
            if k > best_n:
                best_n, best = k, (pts[i], nrm)
        if best is None or best_n < min_inliers:
            break
        p0, nrm = best
        inl = free & (np.abs((pts - p0) @ nrm) < tol)
        c, nrm = _fit_line(pts[inl])
        inl = free & (np.abs((pts - c) @ nrm) < tol)
        if inl.sum() < min_inliers:
            break
        c, nrm = _fit_line(pts[inl])
        lines.append((c, nrm, inl))
        free &= ~inl
    return lines


def _runs(sorted_s, max_gap):
    """Particiona un vector ordenado en tramos sin saltos > max_gap."""
    if sorted_s.size == 0:
        return []
    cuts = np.nonzero(np.diff(sorted_s) > max_gap)[0] + 1
    return np.split(np.arange(sorted_s.size), cuts)


class DockDetector:

    def __init__(self, range_sigma=0.001, seed=0):
        # Todas las tolerancias escalan con el ruido del sensor: en simulacion
        # sigma = 1 mm; en un LiDAR real conviene subirlo (C1: ~1 cm).
        self.sig = float(range_sigma)
        self.rng = np.random.default_rng(seed)

    def _tol(self, base, k):
        return max(base, k * self.sig)

    # ------------------------------------------------------------------ API
    def detect(self, pts, ray_dirs, origin, hint=None):
        """Busca el marcador.

        pts       (N, 2) puntos en el frame del robot, en orden de scan
        ray_dirs  (N, 2) direccion unitaria de cada rayo en ese mismo frame
        origin    (2,)   origen de los rayos (el laser) en ese frame
        hint      Detection previa (en este frame) para desempatar
        """
        if len(pts) < 30:
            return None
        lines = ransac_lines(pts, self.rng, tol=self._tol(0.012, 2.5))
        best = None
        for c, nrm, inl in lines:
            det = self._try_line(pts, ray_dirs, origin, c, nrm, inl, hint)
            if det is not None and (best is None or det.score > best.score):
                best = det
        return best

    # ------------------------------------------------------------ internos
    def _try_line(self, pts, dirs, origin, c, nrm, inl, hint):
        # Normal hacia el lado del sensor: "sobresalir" = h positiva.
        if np.dot(origin - c, nrm) < 0:
            nrm = -nrm
        tan = np.array([-nrm[1], nrm[0]])
        rel = pts - c
        h = rel @ nrm
        s = rel @ tan

        # La pared tiene que estar a cierta distancia del sensor, si no las
        # cajas quedarian dentro del rango minimo o encima del robot.
        if np.dot(origin - c, nrm) < 0.15:
            return None

        s_wall = s[inl]
        lo, hi = s_wall.min() - 0.30, s_wall.max() + 0.30
        band = self._tol(0.030, 3.0)
        prot = (h > BOX_D - band) & (h < BOX_D + band) & (s > lo) & (s < hi)
        pidx = np.nonzero(prot)[0]
        if pidx.size < 2:
            return None

        order = np.argsort(s[pidx])
        ps = s[pidx][order]
        groups = [pidx[order][g] for g in _runs(ps, 0.035)]
        # Un grupo de caja: ancho observado <= ancho real (+ ruido).
        boxes = []
        for g in groups:
            w = s[g].max() - s[g].min()
            if g.size >= 2 and w <= BOX_W + self._tol(0.015, 3.0):
                boxes.append(g)
        if len(boxes) < 2:
            return None

        best = None
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                cand = self._pair(pts, dirs, origin, c, nrm, tan, h, s,
                                  inl, boxes[i], boxes[j], hint)
                if cand is not None and (best is None or
                                         cand.score > best.score):
                    best = cand
        return best

    def _pair(self, pts, dirs, origin, c, nrm, tan, h, s, inl, ga, gb, hint):
        ca, cb = s[ga].mean(), s[gb].mean()
        if ca > cb:
            ga, gb, ca, cb = gb, ga, cb, ca
        sep = cb - ca
        if abs(sep - PITCH) > 0.035:
            return None
        mid = 0.5 * (ca + cb)

        # Nada que sobresalga en el hueco ni justo fuera de las cajas: ahi
        # tiene que verse pared (o nada, si esta ocluido).
        near = np.abs(s - mid)
        in_gap = near < GAP / 2 - 0.012
        hmin = self._tol(0.025, 3.5)
        if np.any(in_gap & (h > hmin) & (h < 0.30)):
            return None
        out = (near > PITCH / 2 + BOX_W / 2 + 0.015) & (near < 0.30)
        if np.any(out & (h > hmin) & (h < 0.30)):
            return None
        if not np.any(in_gap & (np.abs(h) < self._tol(0.015, 3.0))):
            # sin pared visible en el hueco: aceptable solo si hay pared a
            # ambos lados cerca (vista muy oblicua)
            side = inl & (near < 0.40)
            if not (np.any(side & (s < mid)) and np.any(side & (s > mid))):
                return None

        # ---------- refinado: dos rectas paralelas con normal comun
        # Los puntos de las caras LATERALES (muy visibles en vistas oblicuas)
        # tienen h entre 0 y BOX_D y ensuciarian el ajuste de la cara frontal:
        # seleccion estrecha e iterativa alrededor del plano ya ajustado.
        cand_f = np.concatenate([ga, gb])
        cand_w = np.nonzero(np.abs(s - mid) < 0.60)[0]
        n2, t2 = nrm, tan
        wall_c, off = c, BOX_D
        for tol_f, tol_w in ((self._tol(0.010, 3.0), self._tol(0.012, 3.0)),
                             (self._tol(0.005, 2.5), self._tol(0.006, 2.5))):
            rel = pts - wall_c
            hh = rel @ n2
            front = cand_f[np.abs(hh[cand_f] - off) < tol_f]
            wall = cand_w[np.abs(hh[cand_w]) < tol_w]
            if front.size < 3 or wall.size < 6:
                return None
            pf, pw = pts[front], pts[wall]
            stacked = np.vstack([pf - pf.mean(0), pw - pw.mean(0)])
            _, _, vt = np.linalg.svd(stacked, full_matrices=False)
            n2 = vt[1] if np.dot(vt[1], nrm) > 0 else -vt[1]
            t2 = np.array([-n2[1], n2[0]])
            wall_c = pw.mean(0)
            off = float((pf.mean(0) - wall_c) @ n2)
        res = np.concatenate([(pf - pf.mean(0)) @ n2, (pw - wall_c) @ n2])
        rms = float(np.sqrt(np.mean(res ** 2)))
        if abs(off - BOX_D) > self._tol(0.02, 2.0) or rms > self._tol(0.01, 2.0):
            return None

        # Coordenadas refinadas respecto a la pared.
        rel = pts - wall_c
        h2, s2 = rel @ n2, rel @ t2
        o_h = float((origin - wall_c) @ n2)
        o_s = float((origin - wall_c) @ t2)

        edges = []
        for g in (ga, gb):
            gf = g[np.abs(h2[g] - off) < self._tol(0.006, 2.5)]
            if gf.size < 1:
                return None
            lo_e, hi_e = self._box_edges(gf, h2, s2, dirs, n2, t2, off,
                                         o_h, o_s, self._tol(0.004, 2.5))
            edges += [lo_e, hi_e]
        edges = np.array(edges)
        wa, wb = edges[1] - edges[0], edges[3] - edges[2]
        # anchos reconstruidos deben parecer cajas
        wt = self._tol(0.03, 3.0)
        if abs(wa - BOX_W) > wt or abs(wb - BOX_W) > wt:
            return None
        centers_s = np.array([0.5 * (edges[0] + edges[1]),
                              0.5 * (edges[2] + edges[3])])
        # Cada caja da su centro (media de sus dos bordes). El eje del dock es
        # la media de los dos centros: el centro del hueco, promediando cuatro
        # bordes en vez de dos.
        axis_s = float(centers_s.mean())

        wall_pt = wall_c + axis_s * t2
        box_c = np.array([wall_c + cs * t2 + off * n2 for cs in centers_s])

        pitch_err = abs((centers_s[1] - centers_s[0]) - PITCH)
        score = (front.size + 0.2 * wall.size) / (1.0 + 30 * pitch_err) \
            / (1.0 + 100 * rms)
        if hint is not None:
            jump = np.linalg.norm(wall_pt - hint.wall_pt)
            score /= 1.0 + 5.0 * jump
        return Detection(wall_pt=wall_pt, normal=n2, front_offset=off,
                         box_centers=box_c, edges=edges,
                         n_wall=int(wall.size), n_front=int(front.size),
                         residual=rms, score=float(score),
                         debug={'wall_c': wall_c, 'pitch_err': pitch_err})

    @staticmethod
    def _box_edges(g, h, s, dirs, n, t, off, o_h, o_s, side_tol):
        """Bordes (s_min, s_max) de la cara frontal de una caja.

        g son indices de puntos de la cara frontal. Para cada extremo se mira
        el rayo vecino en el scan (indice +-1): la bisectriz de los dos rayos
        cortada con el plano frontal da el borde con error medio nulo.
        """
        N = len(h)
        out = []
        for side in (-1, 1):
            k = g[np.argmin(s[g])] if side < 0 else g[np.argmax(s[g])]
            s_in = s[k]
            best = None
            for nb in (k - 1, k + 1):
                if nb < 0 or nb >= N:
                    continue
                # El vecino debe quedar FUERA de la caja por este lado.
                if (s[nb] - s_in) * side <= 0.001:
                    continue
                if side_tol < h[nb] < off - side_tol and \
                        abs(s[nb] - s_in) < 0.02:
                    # cara lateral visible: su s es el borde
                    best = s[nb]
                    break
                d = dirs[k] + dirs[nb]
                dn = d @ n
                if abs(dn) < 1e-6:
                    continue
                lam = (off - o_h) / dn
                if lam <= 0:
                    continue
                best = o_s + lam * (d @ t)
                break
            if best is None:
                # sin vecino util (borde del scan): el borde queda justo
                # fuera del ultimo punto que toca la caja
                best = s_in + side * 0.002
            out.append(best)
        return out[0], out[1]


# ======================================================================
# Estimacion y control
# ======================================================================

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tf_to_pose2d(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return np.array([t.x, t.y]), yaw


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class DockFilter:
    """Estimacion del dock en odom: punto de pared sobre el eje y angulo de
    la normal. Kalman con modelo estatico e incertidumbre de proceso
    proporcional al movimiento del robot."""

    def __init__(self):
        self.p = None          # punto de la pared sobre el eje (odom)
        self.a = None          # angulo de la normal (odom)
        self.Pp = 0.0          # varianza de posicion (isotropa)
        self.Pa = 0.0          # varianza de angulo
        self.rejects = 0
        self.n_updates = 0

    @property
    def valid(self):
        return self.p is not None

    def predict(self, ds, dth):
        """Inflar incertidumbre por el movimiento desde la ultima vez."""
        if not self.valid:
            return
        self.Pp += (0.01 * ds) ** 2 + (0.02 * abs(dth)) ** 2
        self.Pa += (0.01 * abs(dth)) ** 2 + (0.005 * ds) ** 2

    def update(self, p, a, sp, sa):
        if not self.valid:
            self.p, self.a = p.copy(), a
            self.Pp, self.Pa = sp ** 2, sa ** 2
            self.n_updates = 1
            return True
        dp = p - self.p
        da = wrap(a - self.a)
        gate_p = 4.0 * math.sqrt(self.Pp + sp ** 2) + 0.02
        gate_a = 4.0 * math.sqrt(self.Pa + sa ** 2) + math.radians(2)
        if np.linalg.norm(dp) > gate_p or abs(da) > gate_a:
            self.rejects += 1
            if self.rejects >= 5:
                # varias medidas coherentes contradicen el estado: reiniciar
                self.p, self.a = p.copy(), a
                self.Pp, self.Pa = sp ** 2, sa ** 2
                self.rejects = 0
            return False
        self.rejects = 0
        kp = self.Pp / (self.Pp + sp ** 2)
        ka = self.Pa / (self.Pa + sa ** 2)
        self.p = self.p + kp * dp
        self.a = wrap(self.a + ka * da)
        self.Pp *= (1 - kp)
        self.Pa *= (1 - ka)
        self.n_updates += 1
        return True


class DockLidar(Node):

    def __init__(self):
        super().__init__('dock_lidar')
        # ------------------------------------------------------ parametros
        # Todos se pueden fijar desde el launch (ver solucion.launch.py).
        # Las distancias d_* se miden desde la PARED DETECTADA a lo largo de
        # su normal, nunca en coordenadas del mundo.

        def P(name, default, desc):
            return self.declare_parameter(
                name, default, ParameterDescriptor(description=desc)).value

        # Interfaces
        self.scan_topic = P('scan_topic', '/scan', 'LaserScan de entrada')
        self.cmd_topic = P('cmd_vel_topic', '/cmd_vel', 'Twist de salida')
        self.dock_topic = P('dock_status_topic', '/dock_status',
                            'DockStatus (solo se lee is_docked)')
        self.odom_frame = P('odom_frame', 'odom', 'Frame fijo de odometria')
        self.base_frame = P('base_frame', 'base_link', 'Frame del robot')
        self.pub_markers = P('publish_markers', True,
                             'Publicar ~/markers para RViz')
        self.rate = P('control_rate_hz', 20.0,
                      'Frecuencia del lazo de control [Hz]')
        self.dry_run = P('dry_run', False,
                         'Evaluar el control publicando solo velocidades cero')
        # Percepcion
        sigma = P('range_sigma', 0.01,
                  'Ruido del LiDAR [m]; escala las tolerancias del detector')
        self.max_range = P('max_scan_range', 6.0,
                           'Ignorar lecturas mas lejanas [m]')
        # Velocidades
        self.v_max = P('v_max', 0.14, 'Velocidad lineal maxima [m/s]')
        self.w_max = P('w_max', 0.4, 'Velocidad angular maxima [rad/s]')
        self.v_fine = P('v_fine', 0.05, 'Velocidad en el tramo fino [m/s]')
        self.v_dock = P('v_dock', 0.035,
                        'Velocidad sobre la rampa [m/s]; >=0.03, minimo del firmware')
        self.v_seat = P('v_seat', 0.01, 'Velocidad de asentamiento final [m/s]')
        self.v_backoff = P('v_backoff', 0.12, 'Velocidad de retroceso [m/s]')
        self.w_search = P('w_search', 0.3, 'Giro de busqueda [rad/s]')
        # Ganancias
        self.k_w = P('k_w', 1.0, 'Ganancia proporcional de rumbo')
        self.k_i = P('k_i', 0.0, 'Ganancia integral de rumbo (tramo fino)')
        # Geometria de la maniobra (desde la pared detectada)
        self.d_fine = P('d_fine', 0.75, 'Inicio del tramo fino [m]')
        self.d_slow = P('d_slow', 0.42,
                        'Desde aqui se exige alineacion o se reintenta [m]')
        self.d_dock = P('d_dock', 0.33, 'Inicio del tramo de rampa [m]')
        self.d_floor = P('d_floor', 0.231,
                         'Distancia de parada para esperar el acoplamiento [m]')
        self.d_stage = P('d_stage', 0.75,
                         'Punto de preparacion sobre el eje (STAGE) [m]')
        self.tol_lat = P('align_tol_lat', 0.025,
                         'Error lateral maximo al entrar a d_slow [m]')
        self.tol_head = math.radians(P(
            'align_tol_head_deg', 15.0,
            'Error de rumbo maximo al entrar a d_slow [grados]'))
        # Busqueda / exploracion
        self.start_delay = P('start_delay_s', 1.0,
                             'Espera inicial antes de moverse [s]')
        self.search_turn = math.radians(P(
            'search_turn_deg', 90.0,
            'Giro en SEARCH antes de explorar [grados]'))
        self.explore_step = P('explore_step', 0.8,
                              'Paso maximo de EXPLORE [m]')
        self.explore_timeout = P('explore_timeout_s', 8.0,
                                 'Tiempo maximo por paso de EXPLORE [s]')
        self.clearance = P('obstacle_clearance', 0.45,
                           'Parar EXPLORE si hay algo a menos de esto '
                           'por delante [m]')
        # Choques
        self.hazard_topic = P('hazard_topic', '/hazard_detection',
                              'HazardDetectionVector (solo BUMP)')
        self.react_bumps = P('react_to_bumps', True,
                             'Ante un choque: ESCAPE y reintento')
        self.bump_escape = P('bump_escape_s', 0.8,
                             'Retroceso recto tras un choque [s]')
        self.max_bumps = P('max_bump_retries', 3,
                           'Reintentos por choque; despues se detiene')
        # Final
        self.dock_push = P('dock_push_s', 0.0,
                           'Obsoleto: no se empuja durante la confirmacion')
        self.dock_confirm = P('dock_confirm_s', 2.0,
                              'Tiempo de lecturas true consecutivas, parado [s]')
        self.dock_wait = P('dock_wait_s', 3.0,
                           'Espera parado antes del siguiente asentamiento [s]')
        self.seat_step = P('seat_step', 0.003,
                           'Avance por pulso de asentamiento, medido con odom [m]')
        self.max_seat_steps = P('max_seat_steps', 0,
                                'Maximo de pulsos finales; despues se detiene')
        self.seat_min_distance = P('seat_min_distance', 0.230,
                                   'No seguir asentando por debajo de esta distancia [m]')
        self.seat_timeout = P('seat_timeout_s', 0.6,
                              'Duracion maxima por pulso final [s]')
        self.seat_tol_lat = P('seat_tol_lat', 0.006,
                              'Error lateral maximo para asentamiento recto [m]')
        self.seat_tol_head = math.radians(P(
            'seat_tol_head_deg', 3.0, 'Error angular maximo para asentamiento [grados]'))
        self.dock_timeout = P('dock_timeout_s', 12.0,
                              'Tiempo maximo de avance sobre la rampa [s]')
        # Barrido de rumbo en el contacto: el dock real puede estar girado
        # unos grados respecto al eje de las cajas (medido en el RBOT: ~3 deg).
        self.sweep = P('dock_sweep', True,
                       'Sin contacto al llegar: barrido de rumbo por pulsos')
        self.sweep_max = math.radians(P('sweep_max_deg', 6.0,
                                        'Amplitud maxima del barrido [grados]'))
        self.sweep_step = math.radians(P('sweep_step_deg', 1.5,
                                         'Paso entre objetivos del barrido [grados]'))
        self.sweep_w = P('sweep_w', 0.3,
                         'Giro de cada pulso [rad/s]; >=0.26 por el minimo del firmware')
        self.sweep_hold = P('sweep_hold_s', 1.2,
                            'Pausa quieto tras cada pulso para leer dock_status [s]')
        self.sweep_tol_lat = P('sweep_tol_lat', 0.02,
                               'Error lateral maximo para iniciar el barrido [m]')
        self.sweep_timeout = P('sweep_timeout_s', 25.0,
                               'Duracion maxima del barrido completo [s]')
        self.max_retries = P('max_dock_retries', 2,
                             'Maximo de reintentos, ademas del intento inicial')
        self.retry_timeout = P('retry_timeout_s', 12.0,
                               'Tiempo maximo por maniobra de recuperacion [s]')
        # Recepcion y sellos de tiempo: no seguir con datos antiguos por VPN.
        self.scan_timeout = P('scan_timeout_s', 0.75, 'Edad maxima del scan [s]')
        self.tf_timeout = P('tf_timeout_s', 0.75, 'Edad maxima del TF de odom [s]')
        self.det_timeout = P('detection_timeout_s', 3.0,
                             'Tiempo maximo guiandose sin detectar el marcador [s]')
        self.status_timeout = P('dock_status_timeout_s', 2.5,
                                'Edad maxima de dock_status [s]')
        self.startup_timeout = P('startup_timeout_s', 15.0,
                                 'Espera maxima de datos al arrancar [s]')
        self.run_timeout = P('run_timeout_s', 180.0,
                             'Duracion maxima de la corrida [s]')

        self.check_params()

        self.det = DockDetector(range_sigma=sigma)
        self.filt = DockFilter()
        self.tf_buf = Buffer(cache_time=Duration(seconds=20))
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.is_docked = False
        self.state = 'SEARCH'
        self.state_t = self.now_s()
        self.started_wall = self.wall_s()
        self.state_wall_t = self.started_wall
        self.inputs_ready = False
        self.wait_reason = None
        self.t0 = None
        self.t_docked = None
        self.last_pose = None
        self.last_det_t = None
        # Planos recientes del muro en odom: la distancia normal no debe
        # heredar el sesgo/retardo del Kalman isotropo de posicion del eje.
        self.wall_observations = deque(maxlen=3)
        self.distance_dbg_t = None
        self.last_scan_t = None
        self.last_dock_t = None
        self.last_dock_stamp = None
        self.dock_true_msgs = 0
        self.passive_dock_reported = False
        self.laser_tf = None
        self.last_scan = None      # (puntos en base_link, pos, yaw)
        self.search_rot = 0.0
        self.last_dth = 0.0
        self.explore_goal = None
        self.dbg_t = None
        self.i_err = 0.0
        self.t_ctl = None
        self.bump_pending = None   # frame del sensor que choco
        self.n_bumps = 0
        self.bump_active = False
        self.n_retries = 0
        self.seat_steps = 0
        self.seat_origin = None
        self.seat_start = None
        self.seat_limit_reached = False
        self.seat_limit_reason = None
        self.seat_min_seen = None
        self.wait_origin = None
        self.final_dbg_t = None
        self.sweep_targets = None   # objetivos de rumbo relativos a la pared
        self.sweep_idx = 0
        self.sweep_tries = 0
        self.sweep_phase = None     # 'pulse' o 'hold'
        self.sweep_phase_t = None
        self.sweep_pulse = (0.0, 0.0)   # (signo, duracion)
        self.sweep_start_t = None
        self.sweep_finished = False
        self.sweep_still_head = None   # rumbo quieto antes del ultimo pulso
        self.sweep_last_head = None    # rumbo del ultimo ciclo
        self.sweep_contact = None      # rumbo del ultimo true (fugaz)
        self.sweep_returns = 0

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_mk = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(DockStatus, self.dock_topic, self.on_dock,
                                 qos_profile_sensor_data)
        if self.react_bumps:
            self.create_subscription(HazardDetectionVector,
                                     self.hazard_topic, self.on_hazard,
                                     qos_profile_sensor_data)
        # El watchdog sigue publicando ceros aunque se pare /clock.
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.rate, self.on_timer, clock=self.control_clock)
        self.get_logger().info(
            f'dock_lidar listo | {self.scan_topic} -> {self.cmd_topic} a '
            f'{self.rate:.0f} Hz | v_max {self.v_max:.2f} m/s, w_max '
            f'{self.w_max:.2f} rad/s, v_fine {self.v_fine:.3f}, v_dock '
            f'{self.v_dock:.3f} | range_sigma {sigma:.3f} m')
        self.get_logger().info(
            f'fase final | d_floor={self.d_floor:.3f} m '
            f'seat_min_distance={self.seat_min_distance:.3f} m | '
            f'pulsos={self.max_seat_steps} de {1e3 * self.seat_step:.1f} mm | '
            f'espera={self.dock_wait:.1f} s confirmacion={self.dock_confirm:.1f} s | '
            f'barrido={"+-%.1f deg" % math.degrees(self.sweep_max) if self.sweep else "no"}')
        if self.dry_run:
            self.get_logger().warn('dry_run=true: solo se publicaran comandos cero')

    def check_params(self):
        """Corregir valores imposibles y avisar de los arriesgados."""
        log = self.get_logger()
        positive = ('rate', 'v_max', 'w_max', 'v_fine', 'v_dock', 'v_seat', 'v_backoff',
                    'w_search', 'dock_confirm', 'dock_wait', 'dock_timeout',
                    'retry_timeout', 'scan_timeout', 'tf_timeout', 'det_timeout',
                    'status_timeout', 'startup_timeout', 'run_timeout',
                    'seat_step', 'seat_min_distance', 'seat_timeout',
                    'seat_tol_lat', 'seat_tol_head', 'sweep_max', 'sweep_step',
                    'sweep_w', 'sweep_hold', 'sweep_tol_lat', 'sweep_timeout')
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} debe ser finito y mayor que cero')
        if self.max_retries < 0 or self.max_bumps < 0:
            raise ValueError('Los limites de reintentos no pueden ser negativos')
        if not 0 <= self.max_seat_steps <= 3 or self.seat_step > 0.005:
            raise ValueError('Asentamiento: maximo 3 pulsos de hasta 5 mm')
        if self.seat_timeout > 0.6:
            raise ValueError('seat_timeout_s no puede superar 0.6 s')
        if not 0.23 <= self.seat_min_distance < self.d_floor:
            raise ValueError('Se exige 0.23 <= seat_min_distance < d_floor')
        if self.sweep_max > math.radians(10.0) or self.sweep_step > self.sweep_max:
            raise ValueError('Barrido: maximo 10 grados y paso <= amplitud')
        if self.rate < 5.0:
            log.warn(f'control_rate_hz={self.rate} es muy bajo: el Create 3 '
                     'se detiene si no recibe cmd_vel seguido. Uso 5 Hz.')
            self.rate = 5.0
        for name, limit in (('v_max', MAX_LINEAR_SPEED),
                            ('w_max', MAX_ANGULAR_SPEED)):
            if getattr(self, name) > limit:
                log.warn(f'{name} limitado a {limit:.2f} por Kalman')
                setattr(self, name, limit)
        for name in ('v_fine', 'v_dock', 'v_backoff'):
            setattr(self, name, min(getattr(self, name), self.v_max))
        self.v_seat = min(self.v_seat, self.v_dock, self.v_max, 0.01)
        self.w_search = min(self.w_search, self.w_max)
        self.sweep_w = min(self.sweep_w, self.w_max)
        if self.sweep and self.sweep_w * 0.1175 < 0.03:
            log.warn(f'sweep_w={self.sweep_w:.2f} rad/s: las ruedas quedan bajo '
                     'el minimo de 0.03 m/s del firmware y puede no girar')
        if self.dock_push != 0.0:
            log.warn('dock_push_s se ignora: la confirmacion se hace parado')
        for a, b in (('d_floor', 'd_dock'), ('d_dock', 'd_slow'),
                     ('d_slow', 'd_fine')):
            if getattr(self, a) >= getattr(self, b):
                log.warn(f'{a} deberia ser menor que {b}: la maniobra '
                         'final puede comportarse raro.')
        if self.d_floor < 0.23:
            log.warn(f'd_floor={self.d_floor}: por debajo de ~0.225 m el '
                     'robot toca las cajas.')
        if self.v_dock > 0.05:
            log.warn(f'v_dock={self.v_dock}: sobre la rampa, rapido = rebote.')

    # ---------------------------------------------------------- utilidades
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def wall_s():
        return monotonic()

    def fresh_stamp(self, stamp, timeout):
        seconds = stamp.sec + stamp.nanosec * 1e-9
        return seconds > 0.0 and -0.25 <= self.now_s() - seconds <= timeout

    def set_state(self, s):
        if s != self.state:
            el = self.now_s() - self.t0 if self.t0 else 0.0
            self.get_logger().info(f'[{el:6.2f}s] {self.state} -> {s}')
            self.state = s
            self.state_t = self.now_s()
            self.state_wall_t = self.wall_s()
            if s in ('BACKOFF', 'ESCAPE'):
                self.i_err = 0.0

    def stop(self, reason):
        if self.state != 'STOPPED':
            self.get_logger().error(f'PARADA: {reason}. Relanza para otra corrida.')
            self.set_state('STOPPED')
        self.cmd()

    def retry(self, reason, escape=False):
        if self.n_retries >= self.max_retries:
            self.stop(f'reintentos agotados: {reason}')
            return
        self.n_retries += 1
        self.get_logger().warn(
            f'reintento {self.n_retries}/{self.max_retries}: {reason}')
        self.set_state('ESCAPE' if escape else 'BACKOFF')
        self.cmd()

    def seating_step(self, p, d, e, head_err):
        """Esperar, asentar unos mm o barrer el rumbo. Nunca repetir una
        aproximacion larga."""
        if self.state not in ('WAIT_DOCK', 'SEAT', 'SWEEP'):
            return False
        now = self.wall_s()
        if self.wait_origin is None:
            self.wait_origin = p.copy()
        if self.seat_min_seen is None or d < self.seat_min_seen:
            self.seat_min_seen = d
        total = (0.0 if self.seat_origin is None else
                 float(np.linalg.norm(p - self.seat_origin)))
        budget = min(self.seat_step * self.max_seat_steps, MAX_SEAT_TRAVEL)
        if not self.seat_limit_reached:
            if d <= self.seat_min_distance:
                self.seat_limit_reason = (
                    f'limite de distancia: minimo observado {d:.4f} m '
                    f'<= seat_min_distance={self.seat_min_distance:.4f} m')
            elif total >= budget:
                self.seat_limit_reason = (
                    f'presupuesto de asentamiento: {1e3 * total:.1f}/'
                    f'{1e3 * budget:.1f} mm')
            if self.seat_limit_reason is not None:
                self.seat_limit_reached = True
                self.get_logger().warn(
                    f'asentamiento bloqueado: {self.seat_limit_reason}; '
                    f'pulsos ejecutados={self.seat_steps}/{self.max_seat_steps}')
        if self.final_dbg_t is None or now - self.final_dbg_t >= 0.5:
            self.final_dbg_t = now
            age = float('inf') if self.last_dock_t is None else now - self.last_dock_t
            moved = float(np.linalg.norm(p - self.wait_origin))
            self.get_logger().debug(
                f'final {self.state}: d={d:.4f} min={self.seat_min_seen:.4f} m '
                f'e={1e3 * e:+.1f}mm head={math.degrees(head_err):+.1f}deg '
                f'odom_desde_espera={1e3 * moved:.1f}mm '
                f'pulsos={self.seat_steps}/{self.max_seat_steps} '
                f'is_docked={self.is_docked} edad_estado={age:.2f}s')
        aligned = (abs(e) <= self.seat_tol_lat and
                   abs(head_err) <= self.seat_tol_head)

        if self.state == 'SWEEP':
            return self.sweep_tick(now, d, e, head_err)

        if self.state == 'WAIT_DOCK':
            self.cmd()
            if now - self.state_wall_t < self.dock_wait:
                return True
            seat_next = (aligned and not self.seat_limit_reached and
                         self.seat_steps < self.max_seat_steps and
                         d <= self.d_floor + 0.005)
            if not seat_next and self.sweep_eligible(d, e):
                self.start_sweep(now, head_err)
                return True
            if not aligned:
                self.stop('sin contacto y fuera de tolerancia para asentar')
            elif self.seat_limit_reached:
                self.stop(f'{self.seat_limit_reason}; '
                          f'pulsos ejecutados={self.seat_steps}/{self.max_seat_steps}; '
                          'dock_status=false, no retrocedo')
            elif self.seat_steps >= self.max_seat_steps:
                self.stop(f'pulsos agotados: {self.seat_steps}/{self.max_seat_steps}; '
                          'dock_status=false, no retrocedo')
            elif d > self.d_floor + 0.005:
                self.stop('DOCK sin progreso lejos del contacto; no repito la aproximacion')
            else:
                if self.seat_origin is None:
                    self.seat_origin = p.copy()
                self.seat_start = p.copy()
                self.seat_steps += 1
                self.get_logger().info(
                    f'asentamiento {self.seat_steps}/{self.max_seat_steps}: '
                    f'hasta {1e3 * self.seat_step:.1f} mm a {self.v_seat:.3f} m/s | '
                    f'd={d:.3f} e={1e3 * e:+.1f}mm '
                    f'head={math.degrees(head_err):+.1f}deg dock_status=false')
                self.set_state('SEAT')
            return True

        if not aligned:
            self.stop('se perdio la alineacion durante el asentamiento')
            return True
        if self.seat_start is None:
            self.stop('asentamiento sin pose inicial')
            return True
        travel = float(np.linalg.norm(p - self.seat_start))
        duration = min(self.seat_timeout, self.seat_step / self.v_seat)
        if (self.seat_limit_reached or travel >= self.seat_step or
                now - self.state_wall_t >= duration):
            self.cmd()
            self.get_logger().info(
                f'fin de pulso: avance={1e3 * travel:.1f}mm '
                f'total={1e3 * total:.1f}mm d={d:.3f}; espero dock_status parado')
            self.set_state('WAIT_DOCK')
        else:
            self.cmd(self.v_seat, 0.0)
        return True

    # ------------------------------------------------- barrido de rumbo
    def sweep_eligible(self, d, e):
        """Barrer solo en la distancia de contacto y centrado en el eje.

        Margen de 1.5 cm sobre d_floor: quieto, el robot queda unos mm por
        encima de lo medido en movimiento (latencia), y girar ahi no avanza."""
        return (self.sweep and not self.sweep_finished and
                d <= self.d_floor + 0.015 and abs(e) <= self.sweep_tol_lat)

    def start_sweep(self, now, head_err):
        if self.sweep_targets is None:
            # +1, -1, +2, -2 ... pasos: primero los giros pequenos.
            n = max(1, int(round(self.sweep_max / self.sweep_step)))
            self.sweep_targets = [sign * k * self.sweep_max / n
                                  for k in range(1, n + 1) for sign in (1, -1)]
            self.sweep_idx = 0
            self.sweep_tries = 0
            self.sweep_start_t = now
            self.get_logger().info(
                f'sin contacto en la distancia de acople: barrido de rumbo '
                f'+-{math.degrees(self.sweep_max):.1f} deg en '
                f'{len(self.sweep_targets)} objetivos | rumbo={math.degrees(head_err):+.1f}deg')
        elif self.sweep_contact is not None and self.sweep_returns < 2:
            # Tras un true fugaz, volver primero al rumbo donde llego.
            self.sweep_returns += 1
            self.sweep_targets.insert(self.sweep_idx, self.sweep_contact)
            self.sweep_tries = 0
            self.get_logger().info(
                f'reanudo el barrido: vuelvo al rumbo del ultimo contacto '
                f'{math.degrees(self.sweep_contact):+.1f}deg')
        else:
            self.get_logger().info('reanudo el barrido de rumbo')
        self.sweep_contact = None
        self.set_state('SWEEP')
        # Empezar evaluando quieto: la primera decision usa el rumbo asentado.
        self.sweep_phase = 'hold'
        self.sweep_phase_t = now - self.sweep_hold
        self.cmd()

    def sweep_tick(self, now, d, e, head_err):
        """Pulso de giro corto y pausa quieto. Decidir solo estando parado:
        la latencia de la conexion remota haria pasarse a un giro continuo."""
        if now - self.sweep_start_t > self.sweep_timeout:
            self.sweep_finished = True
            self.stop('barrido de rumbo sin contacto: tiempo agotado; no retrocedo')
            return True
        if abs(e) > self.sweep_tol_lat + 0.01 or d < self.d_floor - 0.02:
            self.sweep_finished = True
            self.stop(f'barrido fuera de envolvente: d={d:.3f} e={1e3 * e:+.1f}mm')
            return True
        self.sweep_last_head = head_err
        if self.sweep_phase == 'pulse':
            sign, duration = self.sweep_pulse
            if now - self.sweep_phase_t < duration:
                self.cmd(0.0, sign * self.sweep_w)
                return True
            self.sweep_phase, self.sweep_phase_t = 'hold', now
            self.cmd()
            return True
        self.cmd()
        if now - self.sweep_phase_t < self.sweep_hold:
            return True
        target = self.sweep_targets[self.sweep_idx]
        err = target - head_err
        if abs(err) <= math.radians(0.5) or self.sweep_tries >= 3:
            # objetivo alcanzado (o no se consigue): siguiente
            self.sweep_idx += 1
            self.sweep_tries = 0
            if self.sweep_idx >= len(self.sweep_targets):
                self.sweep_finished = True
                self.stop(f'barrido de rumbo +-{math.degrees(self.sweep_max):.1f} deg '
                          'sin contacto; no retrocedo')
                return True
            target = self.sweep_targets[self.sweep_idx]
            err = target - head_err
            if abs(err) <= math.radians(0.5):
                self.sweep_phase_t = now
                return True
        self.sweep_tries += 1
        duration = float(np.clip(abs(err) / self.sweep_w, 0.1, 0.5))
        self.sweep_pulse = (math.copysign(1.0, err), duration)
        self.sweep_phase, self.sweep_phase_t = 'pulse', now
        self.sweep_still_head = head_err
        self.get_logger().info(
            f'barrido {self.sweep_idx + 1}/{len(self.sweep_targets)}: objetivo '
            f'{math.degrees(target):+.1f}deg rumbo {math.degrees(head_err):+.1f}deg '
            f'pulso {duration:.2f}s (intento {self.sweep_tries}) d={d:.3f} '
            f'e={1e3 * e:+.1f}mm')
        self.cmd(0.0, self.sweep_pulse[0] * self.sweep_w)
        return True

    def lookup(self, target, source, stamp=None):
        try:
            t = Time() if stamp is None else Time.from_msg(stamp)
            return self.tf_buf.lookup_transform(
                target, source, t, timeout=Duration(seconds=0.0))
        except TransformException:
            if stamp is None:
                return None
            try:
                return self.tf_buf.lookup_transform(target, source, Time())
            except TransformException:
                return None

    # ---------------------------------------------------------- callbacks
    def on_dock(self, msg):
        if not self.fresh_stamp(msg.header.stamp, self.status_timeout):
            self.get_logger().debug('dock_status descartado: sello antiguo o futuro')
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.last_dock_stamp is not None and stamp <= self.last_dock_stamp:
            return
        self.last_dock_stamp = stamp
        now = self.wall_s()
        gap = self.last_dock_t is None or now - self.last_dock_t > self.status_timeout
        self.last_dock_t = now
        was_docked = self.is_docked
        self.is_docked = bool(msg.is_docked)
        if self.state in ('WAIT_DOCK', 'SEAT', 'CONFIRM'):
            self.get_logger().debug(f'dock_status recibido: is_docked={self.is_docked}')
        if self.is_docked:
            if not was_docked:
                self.get_logger().info(f'is_docked=true recibido durante {self.state}')
                if self.state == 'SWEEP':
                    # El estado llega con ~0.2 s de retraso: durante un pulso
                    # corresponde al rumbo quieto previo, no al actual.
                    self.sweep_contact = (self.sweep_still_head
                                          if self.sweep_phase == 'pulse'
                                          else self.sweep_last_head)
            # Cancelar incluso un BACKOFF en cuanto llega el primer true.
            self.cmd()
            self.bump_pending = None
            if self.t_docked is None or gap:
                self.t_docked = now
                self.dock_true_msgs = 0
            self.dock_true_msgs += 1
            if (self.state == 'STOPPED' and not self.passive_dock_reported and
                    self.dock_true_msgs >= 3 and now - self.t_docked >= self.dock_confirm):
                self.passive_dock_reported = True
                self.get_logger().info(
                    'contacto estable observado DESPUES de STOPPED; '
                    'no se reanuda movimiento ni se declara docking autonomo exitoso')
            if self.state not in ('DONE', 'STOPPED'):
                self.set_state('CONFIRM')
        else:
            self.t_docked = None
            self.dock_true_msgs = 0
            self.passive_dock_reported = False

    def on_hazard(self, msg):
        # Solo el parachoques. CLIFF y BACKUP_LIMIT no aplican con
        # safety_override=full en una sala plana. El frame dice que parte
        # del parachoques toco (bump_front_left, bump_right, ...).
        bump = next((d for d in msg.detections
                     if d.type == HazardDetection.BUMP), None)
        was_active, self.bump_active = self.bump_active, bump is not None
        if self.state in ('DONE', 'STOPPED', 'ESCAPE', 'CONFIRM') or self.is_docked:
            return
        if bump is not None and not was_active:
            self.bump_pending = bump.header.frame_id or 'bumper'
            if self.state in ('WAIT_DOCK', 'SEAT', 'SWEEP'):
                self.cmd()

    def on_scan(self, msg):
        if not self.fresh_stamp(msg.header.stamp, self.scan_timeout):
            return
        if self.laser_tf is None:
            tf = self.lookup(self.base_frame, msg.header.frame_id)
            if tf is None:
                return
            self.laser_tf = tf_to_pose2d(tf)
        t_l, yaw_l = self.laser_tf
        R = rot(yaw_l)

        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & \
            (r < min(msg.range_max, self.max_range))
        a = ang[ok]
        dirs = np.stack([np.cos(a), np.sin(a)], 1) @ R.T
        pts = r[ok][:, None] * dirs + t_l
        if len(pts) < 30:
            return

        tf_ob = self.lookup(self.odom_frame, self.base_frame,
                            msg.header.stamp)
        if tf_ob is None or not self.fresh_stamp(tf_ob.header.stamp, self.tf_timeout):
            return
        pos, yaw = tf_to_pose2d(tf_ob)
        Rb = rot(yaw)
        self.last_scan = (pts, pos, yaw)
        self.last_scan_t = self.wall_s()

        hint = None
        if self.filt.valid:
            class _H:
                pass
            hint = _H()
            hint.wall_pt = Rb.T @ (self.filt.p - pos)

        d = self.det.detect(pts, dirs, t_l, hint)
        if d is None:
            return
        # Incertidumbre segun distancia al marcador (de la evaluacion
        # offline del detector: ~1 mm a 1.3 m, ~0.35 mm a 0.4 m).
        rng_m = float(np.linalg.norm(d.wall_pt - t_l))
        sp = 0.0008 + 0.0025 * rng_m + 1.5 * d.residual
        sa = math.radians(0.02 + 0.03 * rng_m) + d.residual

        p_odom = Rb @ d.wall_pt + pos
        n_odom = Rb @ d.normal
        if self.filt.update(p_odom, math.atan2(n_odom[1], n_odom[0]), sp, sa):
            self.last_det_t = self.wall_s()
            self.wall_observations.append(
                (self.last_det_t, p_odom.copy(), n_odom.copy()))
        if self.pub_markers:
            self.publish_markers(d, Rb, pos, msg.header.stamp)

    # ------------------------------------------------------------ control
    def recent_wall_distance(self, p):
        """Mediana de planos recientes evaluados en la pose ACTUAL.

        Se conservan planos en odom, no distancias antiguas: asi el movimiento
        desde el scan queda compensado sin introducir retardo de aproximacion.
        Solo entran detecciones aceptadas por el filtro y no caducadas.
        """
        now = self.wall_s()
        distances = [float((p - point) @ normal)
                     for stamp, point, normal in self.wall_observations
                     if 0.0 <= now - stamp <= self.scan_timeout]
        return None if not distances else float(np.median(distances))

    def robot_pose(self):
        tf = self.lookup(self.odom_frame, self.base_frame)
        if tf is None or not self.fresh_stamp(tf.header.stamp, self.tf_timeout):
            return None
        return tf_to_pose2d(tf)

    def cmd(self, v=0.0, w=0.0):
        m = Twist()
        if not (math.isfinite(v) and math.isfinite(w)):
            v = w = 0.0
        if self.dry_run or self.is_docked or self.state in ('DONE', 'STOPPED'):
            v = w = 0.0
        v_limit = min(self.v_max, MAX_LINEAR_SPEED)
        w_limit = min(self.w_max, MAX_ANGULAR_SPEED)
        m.linear.x = float(np.clip(v, -v_limit, v_limit))
        m.angular.z = float(np.clip(w, -w_limit, w_limit))
        self.pub_cmd.publish(m)

    def on_timer(self):
        wall_now = self.wall_s()
        if self.state == 'STOPPED':
            self.cmd()
            return
        status_fresh = (self.last_dock_t is not None and
                        wall_now - self.last_dock_t <= self.status_timeout)
        if self.state == 'DONE':
            self.cmd()
            if not status_fresh:
                self.stop('se perdio dock_status despues de confirmar')
            elif not self.is_docked:
                self.stop('se perdio el contacto despues de confirmar')
            return
        if wall_now - self.started_wall > self.run_timeout:
            self.stop('tiempo maximo de corrida')
            return

        pose = self.robot_pose()
        problem = None
        if pose is None:
            problem = 'TF de odometria ausente o antiguo'
        elif self.last_scan_t is None or wall_now - self.last_scan_t > self.scan_timeout:
            problem = 'LiDAR ausente o antiguo'
        elif not status_fresh:
            problem = 'dock_status ausente o antiguo'
        elif self.filt.valid and (self.last_det_t is None or
                                 wall_now - self.last_det_t > self.det_timeout):
            problem = 'marcador perdido durante demasiado tiempo'
        if problem is not None:
            self.cmd()
            if self.inputs_ready or wall_now - self.started_wall > self.startup_timeout:
                self.stop(problem)
            elif self.wait_reason != problem:
                self.wait_reason = problem
                self.get_logger().warn(f'esperando datos: {problem}')
            return
        self.inputs_ready = True
        self.wait_reason = None
        now = self.now_s()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('LiDAR, TF y dock_status disponibles, arrancando')
        p, th = pose

        if now - self.t0 < self.start_delay:
            self.cmd()
            return

        if self.last_pose is not None:
            ds = float(np.linalg.norm(p - self.last_pose[0]))
            dth = wrap(th - self.last_pose[1])
            self.filt.predict(ds, dth)
            self.last_dth = dth
        self.last_pose = (p.copy(), th)

        if self.state == 'CONFIRM':
            self.cmd()
            if not self.is_docked:
                self.get_logger().warn('contacto transitorio: sigo parado esperando')
                self.set_state('WAIT_DOCK')
            elif (self.dock_true_msgs >= 3 and self.t_docked is not None and
                  self.last_dock_t - self.t_docked >= self.dock_confirm):
                self.set_state('DONE')
                self.get_logger().info(
                    f'acoplamiento confirmado parado: {self.dock_true_msgs} lecturas, '
                    f'{self.last_dock_t - self.t_docked:.2f} s estables')
            return

        if self.bump_pending is not None:
            where, self.bump_pending = self.bump_pending, None
            if self.state in ('WAIT_DOCK', 'SEAT', 'SWEEP'):
                self.stop(f'parachoques en la fase final ({where}); no retrocedo')
            elif self.n_bumps >= self.max_bumps:
                self.stop('limite de choques alcanzado')
            else:
                self.n_bumps += 1
                self.retry(f'choque en {where} durante {self.state}', escape=True)
            return

        if self.state in ('ESCAPE', 'BACKOFF') and \
                wall_now - self.state_wall_t > self.retry_timeout:
            self.stop('la recuperacion no termino a tiempo')
            return

        if self.state == 'ESCAPE':
            # Recto hacia atras: el choque fue por delante (el parachoques
            # del Create 3 es frontal) y girar podria arrastrar el borde
            # contra lo que toco.
            if wall_now - self.state_wall_t < self.bump_escape:
                self.cmd(-self.v_backoff, 0.0)
                return
            if self.filt.valid:
                self.set_state('BACKOFF')
            else:
                self.set_state('SEARCH')
                self.search_rot = 0.0

        if not self.filt.valid:
            self.search_step(now, p, th)
            return

        W, a = self.filt.p, self.filt.a
        n = np.array([math.cos(a), math.sin(a)])
        t = np.array([-n[1], n[0]])
        d = float((p - W) @ n)
        e = float((p - W) @ t)
        th_wall = wrap(a + math.pi)         # mirando a la pared
        head_err = wrap(th - th_wall)

        d_lidar = self.recent_wall_distance(p)
        if (self.state in ('DOCK', 'WAIT_DOCK', 'SEAT', 'SWEEP') or
                (self.state == 'APPROACH' and
                 (d <= self.d_slow or (d_lidar is not None and d_lidar <= self.d_slow)))):
            if d_lidar is None:
                self.stop('sin plano reciente del muro para el tramo final')
                return
            if self.distance_dbg_t is None or wall_now - self.distance_dbg_t >= 0.5:
                self.distance_dbg_t = wall_now
                self.get_logger().debug(
                    f'distancia final: d_filtro={d:.4f} m d_lidar={d_lidar:.4f} m '
                    f'diferencia={1e3 * (d_lidar - d):+.1f}mm')
            d = d_lidar

        if self.seating_step(p, d, e, head_err):
            return

        st = self.state
        if st in ('SEARCH', 'EXPLORE'):
            st = 'APPROACH' if self.in_cone(d, e, 0.55) else 'STAGE'
            self.set_state(st)

        if st == 'STAGE':
            if self.in_cone(d, e, 0.45):
                self.set_state('APPROACH')
                st = self.state
            else:
                Q = W + n * max(self.d_stage, min(d, 1.2))
                self.go_to(p, th, Q)
                return

        if st in ('APPROACH', 'DOCK'):
            if not self.in_cone(d, e, 0.75) and d > self.d_slow:
                self.set_state('STAGE')
                self.cmd(0.0, 0.0)
                return
            close = d < self.d_slow
            misaligned = abs(e) > self.tol_lat or abs(head_err) > self.tol_head
            if close and misaligned:
                self.retry(f'desalineado: d={d:.3f} e={1e3 * e:+.1f}mm '
                           f'head={math.degrees(head_err):+.1f}deg')
                return
            if d <= self.d_floor or (st == 'DOCK' and
                    wall_now - self.state_wall_t >= self.dock_timeout):
                self.get_logger().info(
                    f'espero dock_status parado: d={d:.3f} e={1e3 * e:+.1f}mm '
                    f'head={math.degrees(head_err):+.1f}deg')
                self.set_state('WAIT_DOCK')
                self.cmd()
                return
            if d < self.d_dock:
                self.set_state('DOCK')
            self.follow_axis(d, e, th, th_wall)
            return

        if self.state == 'BACKOFF':
            # Retroceder mirando a la pared, corrigiendo el angulo, hasta
            # tener espacio para una nueva aproximacion.
            if d > self.d_slow + 0.20:
                self.set_state('APPROACH')
                self.cmd(0.0, 0.0)
                return
            w = self.k_w * wrap(th_wall + math.atan2(-e, 0.3) - th)
            if self.dbg_t is None or now - self.dbg_t > 0.5:
                self.dbg_t = now
                self.get_logger().debug(
                    f'BACKOFF d={d:.3f} e={1e3 * e:+.1f}mm '
                    f'head={math.degrees(head_err):+.1f}deg w={w:+.2f}')
            self.cmd(-self.v_backoff, w)
            return

    def search_step(self, now, p, th):
        """Sin estimacion del dock: girar y, si no aparece, explorar.

        Con 360 grados de LiDAR girar casi no aporta: el marcador solo se
        pierde si esta ocluido o se ve demasiado de canto (mas de ~65 grados
        respecto a la normal de la pared). En ambos casos lo que ayuda es
        cambiar de sitio. El giro corto es solo por si algo montado en el
        robot tapa un sector del LiDAR.
        """
        if self.state == 'EXPLORE':
            to_goal = self.explore_goal - p
            facing = abs(wrap(math.atan2(to_goal[1], to_goal[0]) - th)) < 0.5
            blocked = facing and self.front_blocked()
            timeout = now - self.state_t > self.explore_timeout
            reached = np.linalg.norm(self.explore_goal - p) < 0.08
            if blocked or timeout or reached:
                self.set_state('SEARCH')
                self.search_rot = 0.0
                self.cmd(0.0, 0.0)
            else:
                self.go_to(p, th, self.explore_goal)
            return

        if self.state != 'SEARCH':
            self.set_state('SEARCH')
            self.search_rot = 0.0
        if now - self.t0 < self.start_delay:
            self.cmd(0.0, 0.0)
            return
        if self.last_pose is not None:
            self.search_rot += abs(self.last_dth)
        waited = now - self.state_t > 1.0
        if waited and self.search_rot > self.search_turn and \
                self.last_scan is not None:
            goal = self.explore_target()
            if goal is not None:
                self.explore_goal = goal
                self.set_state('EXPLORE')
                return
        self.cmd(0.0, self.w_search)

    def explore_target(self):
        """Paso hacia el centroide del espacio libre visible.

        Los puntos del scan, en orden angular, forman el poligono de lo que
        el LiDAR ve libre alrededor del robot; su centroide de AREA es el
        centro de la zona despejada (el centroide de los puntos no sirve:
        una pared cercana ocupa medio scan y lo arrastraria hacia ella).
        Ir hacia ahi aleja al robot de las paredes y abre el angulo con el
        que ve el marcador.
        """
        pts, pos, yaw = self.last_scan
        if len(pts) < 20:
            return None
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        P = pts[np.argsort(ang)]
        x, y = P[:, 0], P[:, 1]
        xn, yn = np.roll(x, -1), np.roll(y, -1)
        cross = x * yn - xn * y
        area = 0.5 * cross.sum()
        if abs(area) < 1e-3:
            return None
        c = np.array([((x + xn) * cross).sum(), ((y + yn) * cross).sum()]) \
            / (6.0 * area)
        dist = float(np.linalg.norm(c))
        if dist < 0.15:
            return None
        step = min(self.explore_step, dist)
        return pos + rot(yaw) @ (c / dist * step)

    def front_blocked(self):
        if self.last_scan is None:
            return False
        pts = self.last_scan[0]
        ahead = (pts[:, 0] > 0.0) & (pts[:, 0] < self.clearance) & \
            (np.abs(pts[:, 1]) < 0.20)
        return bool(np.any(ahead))

    def in_cone(self, d, e, slope):
        return abs(e) <= slope * max(d - 0.30, 0.0) + 0.01

    def go_to(self, p, th, Q):
        v = Q - p
        dist = float(np.linalg.norm(v))
        err = wrap(math.atan2(v[1], v[0]) - th)
        w = self.k_w * err
        speed = 0.0 if abs(err) > 0.5 else \
            min(self.v_max, 0.8 * dist + 0.05) * math.cos(err) ** 2
        self.cmd(speed, w)

    def follow_axis(self, d, e, th, th_wall):
        # Pure pursuit sobre el eje: apuntar a un punto del eje a distancia L
        # por delante. L se acorta al acercarse, asi que la correccion lateral
        # se endurece justo cuando el error lateral debe desaparecer.
        #
        # El error lateral decae como exp(-s / L) con la distancia recorrida
        # s, y al llegar el rumbo residual es ~e / L. Por eso L es corto
        # desde lejos y el tramo lento (d_fine -> contacto) es largo: el
        # lateral se consume antes de la rampa y el angulo llega a cero junto
        # con el. L es continuo en d: un salto en L es un salto en el rumbo
        # deseado.
        L = float(np.clip(0.35 * (d - 0.26), 0.05, 0.50))
        if d < self.d_dock:
            # Sobre la rampa el lateral ya no cambia de forma apreciable y
            # el termino e / L solo inclinaria el rumbo final. L crece de
            # forma continua desde d_dock para no saltar el rumbo deseado.
            L = max(L, 0.05 + 3.0 * (self.d_dock - d))
        th_des = wrap(th_wall + math.atan2(e, L))
        err = wrap(th_des - th)
        # Integral solo en el tramo fino: a 2-5 cm/s un error de rumbo de
        # decimas de grado pide una diferencia de rueda de ~1 mm/s que el
        # robot no ejecuta (friccion, rampa). La integral la vence.
        now = self.now_s()
        dt = 0.0 if self.t_ctl is None else min(now - self.t_ctl, 0.1)
        self.t_ctl = now
        if d < self.d_fine:
            # con fuga: evita que lo acumulado en un transitorio empuje de
            # mas cuando el error ya cambio de signo
            self.i_err = float(np.clip(0.97 * self.i_err + err * dt,
                                       -0.02, 0.02))
        else:
            self.i_err = 0.0
        w = self.k_w * err + self.k_i * self.i_err
        if d < self.d_dock:
            v = self.v_dock
        elif d < self.d_fine:
            v = self.v_fine
        else:
            v = float(np.clip(0.7 * (d - self.d_fine) + self.v_fine,
                              self.v_fine, self.v_max))
        if self.dbg_t is None or self.now_s() - self.dbg_t > 0.5:
            self.dbg_t = self.now_s()
            self.get_logger().debug(
                f'd={d:.3f} e={1e3 * e:+.1f}mm head={math.degrees(err):+.2f}deg '
                f'L={L:.2f} v={v:.3f}')
        # no avanzar mientras el rumbo este muy mal: primero girar
        v *= max(0.0, math.cos(err)) ** 4 if abs(err) < 1.0 else 0.0
        self.cmd(v, w)

    # --------------------------------------------------------- depuracion
    def publish_markers(self, det, Rb, pos, stamp):
        ma = MarkerArray()
        n_o = Rb @ det.normal

        def mk(i, typ, frame=None):
            m = Marker()
            m.header.frame_id = frame or self.odom_frame
            m.header.stamp = stamp
            m.ns = 'dock_lidar'
            m.id = i
            m.type = typ
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.lifetime = Duration(seconds=0.5).to_msg()
            return m

        yaw = math.atan2(n_o[1], n_o[0])
        for i, bc in enumerate(det.box_centers):
            c = Rb @ bc + pos - n_o * BOX_D / 2
            m = mk(i, Marker.CUBE)
            m.pose.position.x, m.pose.position.y = float(c[0]), float(c[1])
            m.pose.position.z = 0.19
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = BOX_D, BOX_W, 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.6
            ma.markers.append(m)
        if self.filt.valid:
            W = self.filt.p
            nf = np.array([math.cos(self.filt.a), math.sin(self.filt.a)])
            m = mk(10, Marker.LINE_STRIP)
            m.scale.x = 0.005
            m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0
            for k in (0.0, 2.0):
                q = W + nf * k
                m.points.append(Point(x=float(q[0]), y=float(q[1]), z=0.18))
            m.lifetime = Duration(seconds=0).to_msg()
            ma.markers.append(m)
        self.pub_mk.publish(ma)


def main():
    rclpy.init()
    node = DockLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C del launch: salida limpia, sin traza
    except Exception:
        if rclpy.ok():
            raise
        # El Ctrl-C invalido el contexto durante una publicacion en curso.
    finally:
        try:
            node.cmd(0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
