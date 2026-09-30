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
        BACKOFF   si llega desalineado o demasiado cerca: retroceder y
                  reintentar
        ESCAPE    el parachoques detecto un choque (/hazard_detection):
                  retroceder recto para despegarse y pasar a BACKOFF (o a
                  SEARCH si aun no hay estimacion)
        DONE      acoplado: parar

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
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetection, \
    HazardDetectionVector
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
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
        # Cada caja da su centro; si un borde no pudo triangularse se apoya
        # en el ancho conocido. El eje es la media de los dos centros.
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
                cand = o_s + lam * (d @ t)
                # el borde no puede estar mas alla del punto exterior
                best = cand
                break
            if best is None:
                best = s_in + side * 0.002
            out.append(best)
        # Si un lado no se observa bien (ancho anomalo), usar el ancho real
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
        # Percepcion
        sigma = P('range_sigma', 0.001,
                  'Ruido del LiDAR [m]; escala las tolerancias del detector')
        self.max_range = P('max_scan_range', 6.0,
                           'Ignorar lecturas mas lejanas [m]')
        # Velocidades
        self.v_max = P('v_max', 0.30, 'Velocidad lineal maxima [m/s]')
        self.w_max = P('w_max', 1.5, 'Velocidad angular maxima [rad/s]')
        self.v_fine = P('v_fine', 0.05, 'Velocidad en el tramo fino [m/s]')
        self.v_dock = P('v_dock', 0.025, 'Velocidad sobre la rampa [m/s]')
        self.v_backoff = P('v_backoff', 0.12, 'Velocidad de retroceso [m/s]')
        self.w_search = P('w_search', 0.8, 'Giro de busqueda [rad/s]')
        # Ganancias
        self.k_w = P('k_w', 3.0, 'Ganancia proporcional de rumbo')
        self.k_i = P('k_i', 2.5, 'Ganancia integral de rumbo (tramo fino)')
        # Geometria de la maniobra (desde la pared detectada)
        self.d_fine = P('d_fine', 0.60, 'Inicio del tramo fino [m]')
        self.d_slow = P('d_slow', 0.42,
                        'Desde aqui se exige alineacion o se reintenta [m]')
        self.d_dock = P('d_dock', 0.33, 'Inicio del tramo de rampa [m]')
        self.d_floor = P('d_floor', 0.245,
                         'Distancia minima a la pared; mas cerca retrocede [m]')
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
                           'Reintentos por choque; despues se ignoran')
        # Final
        self.dock_push = P('dock_push_s', 0.4,
                           'Seguir empujando tras is_docked [s]')

        self.check_params()

        self.det = DockDetector(range_sigma=sigma)
        self.filt = DockFilter()
        self.tf_buf = Buffer(cache_time=Duration(seconds=20))
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.is_docked = False
        self.state = 'SEARCH'
        self.state_t = self.now_s()
        self.t0 = None
        self.t_docked = None
        self.last_pose = None
        self.last_det_t = None
        self.done_zero_count = 0
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

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_mk = self.create_publisher(MarkerArray, '~/markers', 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(DockStatus, self.dock_topic, self.on_dock,
                                 10)
        if self.react_bumps:
            self.create_subscription(HazardDetectionVector,
                                     self.hazard_topic, self.on_hazard,
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info(
            f'dock_lidar listo | {self.scan_topic} -> {self.cmd_topic} a '
            f'{self.rate:.0f} Hz | v_max {self.v_max:.2f} m/s, w_max '
            f'{self.w_max:.2f} rad/s, v_fine {self.v_fine:.3f}, v_dock '
            f'{self.v_dock:.3f} | range_sigma {sigma:.3f} m')

    def check_params(self):
        """Corregir valores imposibles y avisar de los arriesgados."""
        log = self.get_logger()
        if self.rate < 5.0:
            log.warn(f'control_rate_hz={self.rate} es muy bajo: el Create 3 '
                     'se detiene si no recibe cmd_vel seguido. Uso 5 Hz.')
            self.rate = 5.0
        if self.v_max > 0.46:
            log.warn(f'v_max={self.v_max} supera el limite del Create 3 '
                     '(0.46 m/s con safety_override=full).')
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

    def set_state(self, s):
        if s != self.state:
            el = self.now_s() - self.t0 if self.t0 else 0.0
            self.get_logger().info(f'[{el:6.2f}s] {self.state} -> {s}')
            self.state = s
            self.state_t = self.now_s()
            if s in ('BACKOFF', 'ESCAPE'):
                self.i_err = 0.0

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
        self.is_docked = bool(msg.is_docked)

    def on_hazard(self, msg):
        # Solo el parachoques. CLIFF y BACKUP_LIMIT no aplican con
        # safety_override=full en una sala plana. El frame dice que parte
        # del parachoques toco (bump_front_left, bump_right, ...).
        if self.state in ('DONE', 'ESCAPE') or self.is_docked:
            return
        for det in msg.detections:
            if det.type == HazardDetection.BUMP:
                self.bump_pending = det.header.frame_id or 'bumper'
                return

    def on_scan(self, msg):
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

        tf_ob = self.lookup(self.odom_frame, self.base_frame,
                            msg.header.stamp)
        if tf_ob is None:
            return
        pos, yaw = tf_to_pose2d(tf_ob)
        Rb = rot(yaw)
        self.last_scan = (pts, pos, yaw)

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
        self.filt.update(p_odom, math.atan2(n_odom[1], n_odom[0]), sp, sa)
        self.last_det_t = self.now_s()
        if self.pub_markers:
            self.publish_markers(d, Rb, pos, msg.header.stamp)

    # ------------------------------------------------------------ control
    def robot_pose(self):
        tf = self.lookup(self.odom_frame, self.base_frame)
        return None if tf is None else tf_to_pose2d(tf)

    def cmd(self, v=0.0, w=0.0):
        m = Twist()
        m.linear.x = float(np.clip(v, -self.v_max, self.v_max))
        m.angular.z = float(np.clip(w, -self.w_max, self.w_max))
        self.pub_cmd.publish(m)

    def on_timer(self):
        pose = self.robot_pose()
        if pose is None:
            return
        now = self.now_s()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('TF disponible, arrancando')
        p, th = pose

        if self.last_pose is not None:
            ds = float(np.linalg.norm(p - self.last_pose[0]))
            dth = wrap(th - self.last_pose[1])
            self.filt.predict(ds, dth)
            self.last_dth = dth
        self.last_pose = (p.copy(), th)

        if self.state == 'DONE':
            # unos ceros para dejarlo quieto y luego silencio
            if self.done_zero_count < 20:
                self.cmd(0.0, 0.0)
                self.done_zero_count += 1
            return

        if self.is_docked and self.state != 'DONE':
            if self.t_docked is None:
                self.t_docked = now
                self.get_logger().info(
                    f'is_docked = true a los {now - self.t0:.2f} s')
            if now - self.t_docked >= self.dock_push:
                self.set_state('DONE')
                self.cmd(0.0, 0.0)
                return
        else:
            self.t_docked = None

        if self.bump_pending is not None:
            where, self.bump_pending = self.bump_pending, None
            if self.n_bumps < self.max_bumps:
                self.n_bumps += 1
                self.get_logger().warn(
                    f'choque en {where} durante {self.state} '
                    f'({self.n_bumps}/{self.max_bumps}): retrocedo y '
                    'reintento')
                self.set_state('ESCAPE')
            elif self.n_bumps == self.max_bumps:
                self.n_bumps += 1
                self.get_logger().warn(
                    'demasiados choques: los ignoro y sigo (puede ser el '
                    'contacto con la rampa del dock)')

        if self.state == 'ESCAPE':
            # Recto hacia atras: el choque fue por delante (el parachoques
            # del Create 3 es frontal) y girar podria arrastrar el borde
            # contra lo que toco.
            if now - self.state_t < self.bump_escape:
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

        st = self.state
        if st in ('SEARCH', 'EXPLORE'):
            st = 'APPROACH' if self.in_cone(d, e, 0.55) else 'STAGE'
            self.set_state(st)

        if st == 'STAGE':
            if self.in_cone(d, e, 0.45):
                self.set_state('APPROACH')
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
            if (close and misaligned) or d < self.d_floor:
                self.set_state('BACKOFF')
            else:
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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
