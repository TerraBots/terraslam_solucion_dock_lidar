"""Control real con reloj/TF/publicadores en memoria: sin ROS ni movimiento."""
import math
from pathlib import Path
import runpy
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped, Twist
from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
from rclpy.qos import QoSCompatibility, qos_check_compatible, qos_profile_sensor_data

from terraslam_solucion_dock_lidar import dock_lidar as module


# Los tests de asentamiento verifican la logica de pulsos con la geometria en
# la que se diseno (tope 0.245, minimo 0.235); los defaults fisicos cambiaron.
SEAT_GEOMETRY = {'d_floor': 0.245, 'seat_min_distance': 0.235}


def stamp(seconds):
    ns = round(seconds * 1e9)
    return SimpleNamespace(sec=ns // 10**9, nanosec=ns % 10**9)


class Harness(module.DockLidar):
    def __init__(self, **overrides):
        self.ros_now = self.wall_now = 100.0
        self.overrides = overrides
        self.messages = []
        self.logs = []
        self.recorded_qos = {}
        self.tf = TransformStamped()
        self.tf.transform.rotation.w = 1.0
        self.refresh_tf()
        super().__init__()

    def now_s(self):
        return self.ros_now

    def wall_s(self):
        return self.wall_now

    def declare_parameter(self, name, default, descriptor):
        return SimpleNamespace(value=self.overrides.get(name, default))

    def get_logger(self):
        return SimpleNamespace(**{level: self.logs.append
                                 for level in ('info', 'warn', 'error', 'debug')})

    def create_publisher(self, msg_type, topic, qos):
        return SimpleNamespace(publish=self.messages.append if msg_type is Twist
                               else lambda msg: None)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.recorded_qos[topic] = qos

    def create_timer(self, period, callback, *, clock):
        self.timer_clock = clock

    def refresh_tf(self):
        s = stamp(self.ros_now)
        self.tf.header.stamp.sec, self.tf.header.stamp.nanosec = s.sec, s.nanosec

    def status(self, value):
        msg = DockStatus()
        s = stamp(self.ros_now)
        msg.header.stamp.sec, msg.header.stamp.nanosec = s.sec, s.nanosec
        msg.is_docked = value
        self.on_dock(msg)

    def ready(self, state='DOCK', distance=None, lateral=0.002):
        distance = self.d_floor if distance is None else distance
        self.filt.update(np.array([distance, lateral]), math.pi, 0.005, 0.005)
        self.last_scan_t = self.last_det_t = self.wall_now
        self.refresh_wall_observations()
        self.status(False)
        self.inputs_ready = True
        self.t0 = self.ros_now - self.start_delay - 1.0
        self.set_state(state)
        return self

    def refresh_wall_observations(self):
        # Estos escenarios de estados suponen percepcion ideal y coherente.
        # Las pruebas especificas de distancia inyectan observaciones distintas.
        if self.filt.valid:
            normal = np.array([math.cos(self.filt.a), math.sin(self.filt.a)])
            self.wall_observations.clear()
            for _ in range(3):
                self.wall_observations.append((self.wall_now, self.filt.p.copy(), normal.copy()))

    def advance(self, seconds, status=False, scan=True, tf=True, detection=True):
        self.ros_now += seconds
        self.wall_now += seconds
        if tf:
            self.refresh_tf()
        if scan:
            self.last_scan_t = self.wall_now
        if detection:
            self.last_det_t = self.wall_now
            if scan and tf:
                self.refresh_wall_observations()
        if status is not None:
            self.status(status)
        self.on_timer()

    def last_command(self):
        return self.messages[-1].linear.x, self.messages[-1].angular.z


@pytest.fixture
def make_node(monkeypatch):
    # Ejecutar __init__ y los callbacks de produccion sin crear contexto DDS.
    monkeypatch.setattr(module.Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(module, 'Buffer', lambda **kw: SimpleNamespace())
    monkeypatch.setattr(module, 'TransformListener', lambda *a: None)

    def make(**overrides):
        node = Harness(**overrides)
        node.tf_buf.lookup_transform = lambda *a, **kw: node.tf
        return node
    return make


def test_floor_waits_without_reverse_and_confirms_late_status(make_node):
    node = make_node().ready()
    node.on_timer()
    assert node.state == 'WAIT_DOCK'
    assert node.last_command() == (0.0, 0.0)
    node.advance(1.0, status=True)
    assert node.state == 'CONFIRM'
    node.advance(1.0, status=True)
    assert node.state == 'CONFIRM'
    node.advance(1.0, status=True)
    assert node.state == 'DONE'
    assert all(msg.linear.x == msg.angular.z == 0.0 for msg in node.messages)


def test_true_during_backoff_stops_immediately_and_false_is_not_done(make_node):
    node = make_node().ready('BACKOFF', distance=0.4)
    node.on_timer()
    assert node.last_command()[0] < 0.0
    node.ros_now += 0.1
    node.status(True)
    assert node.last_command() == (0.0, 0.0)  # sin esperar al timer
    node.advance(0.5, status=False)
    assert node.state == 'WAIT_DOCK'
    assert node.last_command() == (0.0, 0.0)


def test_single_true_never_confirms_on_timer_alone(make_node):
    node = make_node().ready()
    node.ros_now += 0.1
    node.status(True)
    node.advance(2.1, status=None)
    assert node.state == 'CONFIRM'
    node.advance(0.5, status=None)
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


def test_duplicate_status_cannot_supply_confirmation_messages(make_node):
    node = make_node().ready()
    node.ros_now += 0.1
    node.status(True)
    node.status(True)
    node.status(True)
    assert node.dock_true_msgs == 1


def test_false_resets_the_entire_confirmation_interval(make_node):
    node = make_node().ready()
    node.advance(0.1, status=True)
    node.advance(1.5, status=True)
    node.advance(0.1, status=False)
    assert node.state == 'WAIT_DOCK'
    node.advance(0.1, status=True)
    node.advance(1.0, status=True)
    assert node.state == 'CONFIRM'
    node.advance(1.0, status=True)
    assert node.state == 'DONE'
    node.advance(0.1, status=False)
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


@pytest.mark.parametrize('lost,timeout', [
    ('scan', 0.8), ('tf', 0.8), ('detection', 3.1), ('status', 2.6),
])
def test_lost_inputs_stop_and_never_resume_automatically(make_node, lost, timeout):
    node = make_node().ready('APPROACH', distance=1.0)
    node.on_timer()
    assert node.last_command()[0] > 0.0
    node.advance(timeout, status=None if lost == 'status' else False,
                 scan=lost != 'scan', tf=lost != 'tf', detection=lost != 'detection')
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)
    node.advance(0.1)
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


def test_wall_watchdog_stops_even_if_ros_clock_freezes(make_node):
    node = make_node().ready('APPROACH', distance=1.0)
    node.wall_now += 1.0
    node.on_timer()
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)
    assert node.timer_clock.clock_type == module.ClockType.STEADY_TIME


def test_startup_waits_and_start_delay_applies_with_marker(make_node):
    node = make_node(start_delay_s=5.0)
    node.on_timer()
    assert node.last_command() == (0.0, 0.0)
    node.ready('APPROACH', distance=1.0)
    node.t0 = None
    node.on_timer()
    assert node.last_command() == (0.0, 0.0)
    node.advance(4.0)
    assert node.last_command() == (0.0, 0.0)
    node.advance(1.0)
    assert node.last_command()[0] > 0.0


def test_startup_and_total_run_have_finite_limits(make_node):
    node = make_node()
    node.advance(16.0, status=None, scan=False, tf=False, detection=False)
    assert node.state == 'STOPPED'
    node = make_node().ready('APPROACH', distance=1.0)
    node.advance(181.0)
    assert node.state == 'STOPPED'


def test_misalignment_still_uses_only_bounded_retries(make_node):
    node = make_node(max_dock_retries=1).ready('APPROACH', distance=0.4, lateral=0.03)
    node.on_timer()
    assert node.state == 'BACKOFF'
    assert node.n_retries == 1
    assert node.last_command() == (0.0, 0.0)
    node.advance(0.1)
    assert node.last_command()[0] < 0.0
    # Tras otro intento fallido, agota el presupuesto y queda parado.
    node.set_state('APPROACH')
    node.on_timer()
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


def test_logged_floor_sequence_does_not_repeat_long_backoffs(make_node):
    node = make_node(max_seat_steps=3, dock_sweep=False, **SEAT_GEOMETRY).ready(
        'APPROACH', distance=0.333, lateral=-0.0022)
    for distance in (0.333, 0.315, 0.300, 0.283, 0.273, 0.259, 0.244):
        node.filt.p[0] = distance
        node.advance(0.5)
    assert node.state == 'WAIT_DOCK'
    assert node.n_retries == 0
    for step in range(3):
        node.advance(3.1)
        assert node.state == 'SEAT'
        node.advance(0.05)
        assert node.last_command() == (0.01, 0.0)
        node.tf.transform.translation.x = 0.0031 * (step + 1)
        node.advance(0.1)
        assert node.state == 'WAIT_DOCK'
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert node.seat_steps == 3
    assert node.n_retries == 0
    assert all(msg.linear.x >= 0.0 for msg in node.messages)


def test_status_during_seating_stops_and_confirms(make_node):
    node = make_node(max_seat_steps=3).ready()
    node.on_timer()
    node.advance(3.1)
    node.advance(0.05)
    assert node.last_command()[0] == 0.01
    node.ros_now += 0.1
    node.status(True)
    assert node.state == 'CONFIRM'
    assert node.last_command() == (0.0, 0.0)
    node.advance(1.0, status=True)
    node.advance(1.0, status=True)
    assert node.state == 'DONE'


def test_stationary_odometry_cannot_cause_unlimited_pushing(make_node):
    node = make_node(max_seat_steps=3, dock_sweep=False, **SEAT_GEOMETRY).ready()
    node.on_timer()
    for _ in range(3):
        node.advance(3.1)
        assert node.state == 'SEAT'
        node.advance(0.05)
        assert node.last_command()[0] == 0.01
        node.advance(0.26)  # tambien limita por seat_step/v_seat = 0.3 s
        assert node.state == 'WAIT_DOCK'
        assert node.last_command() == (0.0, 0.0)
    node.advance(3.1)
    assert node.state == 'STOPPED'


def test_total_seating_budget_includes_motion_while_waiting(make_node):
    node = make_node(max_seat_steps=3, dock_sweep=False, **SEAT_GEOMETRY).ready()
    node.on_timer()
    node.advance(3.1)
    node.advance(0.05)
    node.tf.transform.translation.x = 0.0031
    node.advance(0.1)
    assert node.state == 'WAIT_DOCK'
    node.tf.transform.translation.x = 0.0091  # inercia/deslizamiento tras parar
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert node.seat_steps == 1


def test_larger_pulses_still_obey_the_absolute_travel_budget(make_node):
    node = make_node(max_seat_steps=3, seat_step=0.005, seat_min_distance=0.230,
                     dock_sweep=False, d_floor=0.245).ready()
    node.on_timer()
    for step in range(2):
        node.advance(3.1)
        assert node.state == 'SEAT'
        node.advance(0.05)
        node.tf.transform.translation.x = 0.0051 * (step + 1)
        node.advance(0.1)
        assert node.state == 'WAIT_DOCK'
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert node.seat_steps == 2


@pytest.mark.parametrize('distance,lateral', [(0.2349, 0.002), (0.244, 0.007), (0.28, 0.002)])
def test_seating_does_not_start_outside_its_envelope(make_node, distance, lateral):
    node = make_node(max_seat_steps=3, dock_sweep=False, **SEAT_GEOMETRY).ready(
        'WAIT_DOCK', distance=distance, lateral=lateral)
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert node.seat_steps == 0
    assert node.last_command() == (0.0, 0.0)


def test_seating_distance_limit_is_latched_even_if_estimate_rebounds(make_node):
    node = make_node(max_seat_steps=3, dock_sweep=False, **SEAT_GEOMETRY).ready()
    node.on_timer()
    node.advance(3.1)
    node.filt.p[0] = 0.2349
    node.advance(0.05)
    assert node.state == 'WAIT_DOCK'
    node.filt.p[0] = 0.240
    node.advance(3.1)
    assert node.state == 'STOPPED'


def test_distance_guard_before_first_pulse_reports_zero_pulses(make_node):
    node = make_node(max_seat_steps=1, dock_sweep=False, **SEAT_GEOMETRY).ready(distance=0.244)
    node.on_timer()
    node.advance(0.2)
    node.filt.p[0] = 0.2349
    node.advance(0.1)
    node.filt.p[0] = 0.244
    node.advance(2.9)
    assert node.state == 'STOPPED'
    assert node.seat_steps == 0
    assert node.seat_min_seen == pytest.approx(0.2349)
    assert 'limite de distancia' in node.seat_limit_reason
    assert any('pulsos ejecutados=0/1' in text for text in node.logs)
    assert any('odom_desde_espera=0.0mm' in text for text in node.logs)
    assert all(msg.linear.x == msg.angular.z == 0.0 for msg in node.messages)


def test_late_contact_is_observed_without_rearming_after_stop(make_node):
    node = make_node().ready()
    node.stop('prueba de observacion pasiva')
    node.advance(0.1, status=True, scan=False, tf=False, detection=False)
    assert not node.passive_dock_reported
    node.advance(1.0, status=True, scan=False, tf=False, detection=False)
    node.advance(1.0, status=True, scan=False, tf=False, detection=False)
    assert node.passive_dock_reported
    assert node.state == 'STOPPED'
    reports = [text for text in node.logs if 'DESPUES de STOPPED' in text]
    assert len(reports) == 1
    node.advance(1.0, status=True, scan=False, tf=False, detection=False)
    assert len([text for text in node.logs if 'DESPUES de STOPPED' in text]) == 1
    node.advance(0.1, status=False, scan=False, tf=False, detection=False)
    assert not node.passive_dock_reported
    assert all(msg.linear.x == msg.angular.z == 0.0 for msg in node.messages)


def test_seating_stops_if_heading_alignment_is_lost(make_node):
    node = make_node(max_seat_steps=3).ready()
    node.on_timer()
    node.advance(3.1)
    angle = math.radians(4.0)
    node.tf.transform.rotation.z = math.sin(angle / 2)
    node.tf.transform.rotation.w = math.cos(angle / 2)
    node.advance(0.05)
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


def test_seating_bump_stops_without_automatic_backoff(make_node):
    node = make_node(max_seat_steps=3).ready()
    node.on_timer()
    node.advance(3.1)
    node.advance(0.05)
    bump = HazardDetection()
    bump.type = HazardDetection.BUMP
    bump.header.frame_id = 'bump_front_center'
    msg = HazardDetectionVector()
    msg.detections = [bump]
    node.on_hazard(msg)
    assert node.last_command() == (0.0, 0.0)
    node.advance(0.05)
    assert node.state == 'STOPPED'
    assert node.n_retries == 0


def test_disabled_seating_stops_without_reversing(make_node):
    node = make_node(max_seat_steps=0, dock_sweep=False, **SEAT_GEOMETRY).ready()
    node.on_timer()
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert all(msg.linear.x == msg.angular.z == 0.0 for msg in node.messages)


@pytest.mark.parametrize('state', ['BACKOFF', 'ESCAPE'])
def test_recovery_timeout_stops(make_node, state):
    node = make_node().ready(state, distance=0.4)
    node.advance(12.1)
    assert node.state == 'STOPPED'


def test_stalled_dock_waits_instead_of_pushing_indefinitely(make_node):
    node = make_node().ready(distance=0.28)
    node.advance(12.1)
    assert node.state == 'WAIT_DOCK'
    assert node.last_command() == (0.0, 0.0)


def test_persistent_bump_is_one_event_and_limit_stops(make_node):
    node = make_node(max_bump_retries=0).ready('APPROACH', distance=1.0)
    msg = HazardDetectionVector()
    bump = HazardDetection()
    bump.type = HazardDetection.BUMP
    bump.header.frame_id = 'bump_front_center'
    msg.detections = [bump]
    node.on_hazard(msg)
    node.on_timer()
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)
    node = make_node().ready('APPROACH', distance=1.0)
    node.on_hazard(msg)
    node.on_timer()
    assert node.state == 'ESCAPE'
    node.advance(0.9)
    node.on_hazard(msg)
    assert node.bump_pending is None
    assert node.n_retries == 1


@pytest.mark.parametrize('state', [
    'SEARCH', 'EXPLORE', 'STAGE', 'APPROACH', 'DOCK', 'BACKOFF', 'ESCAPE',
    'WAIT_DOCK', 'SEAT', 'CONFIRM', 'DONE', 'STOPPED',
])
@pytest.mark.parametrize('dry_run', [False, True])
def test_limits_in_every_state_and_dry_run(make_node, state, dry_run):
    node = make_node(v_max=9.0, w_max=9.0, v_fine=9.0, v_dock=9.0, v_seat=9.0,
                     v_backoff=9.0, w_search=9.0, dry_run=dry_run, max_seat_steps=3)
    node.ready(state, distance=0.245 if state == 'SEAT' else
               0.28 if state in ('DOCK', 'BACKOFF') else 1.0)
    if state == 'SEAT':
        node.seat_origin = node.seat_start = np.zeros(2)
        node.seat_steps = 1
    if state in ('SEARCH', 'EXPLORE'):
        node.filt = module.DockFilter()
        node.last_scan = (np.empty((0, 2)), np.zeros(2), 0.0)
        node.explore_goal = np.array([0.8, 0.0])
    if state == 'STAGE':
        node.filt.p[1] = 0.9
    if state == 'CONFIRM':
        node.advance(0.1, status=True)
    else:
        node.on_timer()
    assert node.messages
    for msg in node.messages:
        assert abs(msg.linear.x) <= 0.20
        assert abs(msg.angular.z) <= 0.50
        if dry_run:
            assert msg.linear.x == msg.angular.z == 0.0
    node.cmd(99.0, -99.0)
    assert abs(node.last_command()[0]) <= 0.20
    assert abs(node.last_command()[1]) <= 0.50


def test_nonfinite_command_is_zero(make_node):
    node = make_node()
    node.cmd(float('nan'), 1.0)
    assert node.last_command() == (0.0, 0.0)


def test_biased_filter_does_not_stop_before_reaching_the_wall_limit(make_node):
    # Reproduce la corrida: el filtro cree 244 mm pero los scans dan 260 mm
    # (con el tope de entonces, 0.245 m).
    node = make_node(d_floor=0.245).ready('DOCK', distance=0.244)
    node.wall_observations.clear()
    for distance in (0.259, 0.260, 0.261):
        node.wall_observations.append((node.wall_now, np.array([distance, 0.0]),
                                       np.array([-1.0, 0.0])))
    node.on_timer()
    assert node.state == 'DOCK'
    assert node.last_command()[0] > 0.0
    # Avanza 16 mm en odom: los mismos planos ahora situan el centro en 244 mm.
    node.tf.transform.translation.x = 0.016
    node.on_timer()
    assert node.state == 'WAIT_DOCK'
    assert node.last_command() == (0.0, 0.0)


def test_recent_wall_distance_compensates_current_odometry_without_distance_lag(make_node):
    node = make_node().ready()
    node.wall_observations.clear()
    for age in (0.2, 0.1, 0.0):
        node.wall_observations.append((node.wall_now-age, np.array([1.0, 0.0]),
                                       np.array([-1.0, 0.0])))
    assert node.recent_wall_distance(np.array([0.75, 0.0])) == pytest.approx(0.25)


def test_lidar_proximity_guard_also_handles_filter_bias_away_from_wall(make_node):
    node = make_node(d_floor=0.245).ready('APPROACH', distance=0.5)
    node.wall_observations.clear()
    for _ in range(3):
        node.wall_observations.append((node.wall_now, np.array([0.24, 0.0]),
                                       np.array([-1.0, 0.0])))
    node.on_timer()
    assert node.state == 'WAIT_DOCK'
    assert node.last_command() == (0.0, 0.0)


def test_recent_wall_distance_is_robust_to_one_bad_plane_and_drops_stale_planes(make_node):
    node = make_node().ready()
    node.wall_observations.clear()
    for distance in (0.260, 0.261, 0.500):
        node.wall_observations.append((node.wall_now, np.array([distance, 0.0]),
                                       np.array([-1.0, 0.0])))
    assert node.recent_wall_distance(np.zeros(2)) == pytest.approx(0.261)
    node.wall_now += node.scan_timeout + 0.01
    assert node.recent_wall_distance(np.zeros(2)) is None


def test_final_approach_stops_if_recent_wall_planes_are_missing(make_node):
    node = make_node().ready('DOCK', distance=0.260)
    node.wall_observations.clear()
    node.on_timer()
    assert node.state == 'STOPPED'
    assert node.last_command() == (0.0, 0.0)


def test_scan_only_records_wall_planes_when_the_measurement_is_accepted(make_node):
    node = make_node(publish_markers=False)
    node.laser_tf = (np.zeros(2), 0.0)
    msg = module.LaserScan()
    msg.header.stamp = node.tf.header.stamp
    msg.header.frame_id = 'laser_link'
    msg.angle_min = -math.pi
    msg.angle_increment = 2*math.pi/720
    msg.range_min, msg.range_max = 0.15, 12.0
    msg.ranges = [2.0]*720
    detection = SimpleNamespace(wall_pt=np.array([0.260, 0.0]),
                                normal=np.array([-1.0, 0.0]), residual=0.001)
    node.det.detect = lambda *args: detection
    node.filt.update = lambda *args: False
    node.on_scan(msg)
    assert not node.wall_observations
    node.filt.update = lambda *args: True
    node.on_scan(msg)
    assert len(node.wall_observations) == 1
    assert node.recent_wall_distance(np.zeros(2)) == pytest.approx(0.260)


def test_status_qos_accepts_hardware_best_effort(make_node):
    node = make_node()
    compatibility, _ = qos_check_compatible(qos_profile_sensor_data,
                                            node.recorded_qos['/dock_status'])
    assert compatibility != QoSCompatibility.ERROR
    assert node.recorded_qos['/dock_status'].reliability == qos_profile_sensor_data.reliability


def test_launch_defaults_match_node_and_only_one_node_is_launched(make_node, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    path = Path(__file__).resolve().parents[1] / 'launch' / 'solucion.launch.py'
    launch = runpy.run_path(str(path))
    seen = {}
    original = Harness.declare_parameter
    # Comparar ambos contratos sin inicializar ROS ni publicar por DDS.
    Harness.declare_parameter = lambda self, name, default, descriptor: (
        seen.setdefault(name, default), original(self, name, default, descriptor))[1]
    try:
        make_node()
    finally:
        Harness.declare_parameter = original
    for name, default, typ, _ in launch['PARAMS']:
        if name == 'use_sim_time':
            assert default == 'false'
            continue
        value = default == 'true' if typ is bool else typ(default)
        assert seen[name] == value
    actions = launch['generate_launch_description']().entities
    assert sum(isinstance(a, launch['Node']) for a in actions) == 1


# ---------------------------------------------------------------- barrido
def set_yaw(node, yaw):
    node.tf.transform.rotation.z = math.sin(yaw / 2)
    node.tf.transform.rotation.w = math.cos(yaw / 2)


def start_sweep(node):
    node.on_timer()
    assert node.state == 'WAIT_DOCK'
    node.advance(3.1)
    assert node.state == 'SWEEP'
    assert node.last_command() == (0.0, 0.0)
    return len(node.messages)


def test_sweep_rotates_in_place_by_pulses_and_confirms_contact(make_node):
    # RBOT: llega centrado a la distancia de acople pero el dock esta girado.
    node = make_node().ready()
    first = start_sweep(node)
    node.advance(0.05)
    assert node.last_command() == (0.0, pytest.approx(0.3))   # objetivo +1.5 deg
    node.advance(0.12)
    assert node.last_command() == (0.0, 0.0)                  # pausa quieto
    set_yaw(node, math.radians(1.5))
    node.advance(1.25)
    assert node.last_command() == (0.0, pytest.approx(-0.3))  # objetivo -1.5 deg
    node.advance(0.12)
    set_yaw(node, math.radians(-1.5))
    node.advance(0.5, status=True)                            # contacto en la pausa
    assert node.state == 'CONFIRM'
    assert node.last_command() == (0.0, 0.0)
    node.advance(1.0, status=True)
    node.advance(1.0, status=True)
    assert node.state == 'DONE'
    assert all(msg.linear.x == 0.0 for msg in node.messages[first:])
    assert all(abs(msg.angular.z) <= 0.3 + 1e-9 for msg in node.messages)


def test_true_during_a_pulse_stops_rotation_immediately(make_node):
    node = make_node().ready()
    start_sweep(node)
    node.advance(0.05)
    assert node.last_command()[1] != 0.0
    node.ros_now += 0.02
    node.status(True)
    assert node.state == 'CONFIRM'
    assert node.last_command() == (0.0, 0.0)


def test_transient_true_returns_to_the_still_heading_where_it_arrived(make_node):
    # RBOT 2026-11-02 23:11: true fugaz al iniciar el pulso hacia +3 desde
    # -1.1 deg; al reanudar, el barrido seguia hacia +3, lejos del contacto.
    node = make_node().ready()
    start_sweep(node)
    node.advance(0.12)
    set_yaw(node, math.radians(1.5))
    node.advance(1.25)                                         # pulso hacia -1.5
    node.advance(0.12)
    set_yaw(node, math.radians(-1.5))
    node.advance(1.25)                                         # pulso hacia +3
    assert node.last_command() == (0.0, pytest.approx(0.3))
    node.ros_now += 0.02
    node.status(True)                                          # llega con retraso
    assert node.state == 'CONFIRM'
    set_yaw(node, math.radians(0.5))                           # el pulso alcanzo a girar
    node.advance(1.0, status=False)                            # true fugaz
    assert node.state == 'WAIT_DOCK'
    node.advance(3.1)
    assert node.state == 'SWEEP'
    assert any('ultimo contacto -1.5deg' in str(msg) for msg in node.logs)
    node.advance(0.05)
    assert node.last_command() == (0.0, pytest.approx(-0.3))   # vuelve hacia -1.5
    set_yaw(node, math.radians(-1.5))
    node.advance(0.5, status=True)
    node.advance(1.0, status=True)
    node.advance(1.0, status=True)
    assert node.state == 'DONE'
    assert node.sweep_returns == 1


def test_exhausted_sweep_stops_without_reversing(make_node):
    node = make_node().ready()
    first = start_sweep(node)
    for _ in range(600):
        if node.state != 'SWEEP':
            break
        if node.sweep_phase == 'hold':          # el robot ejecuta cada pulso
            set_yaw(node, node.sweep_targets[node.sweep_idx])
        node.advance(0.1)
    assert node.state == 'STOPPED'
    assert node.sweep_finished
    assert any('barrido de rumbo' in str(msg) for msg in node.logs)
    tail = node.messages[first:]
    assert all(msg.linear.x == 0.0 for msg in tail)
    assert max(abs(msg.angular.z) for msg in tail) == pytest.approx(0.3)
    assert node.last_command() == (0.0, 0.0)


def test_sweep_never_exceeds_its_amplitude_or_time_budget(make_node):
    node = make_node(sweep_timeout_s=5.0).ready()
    start_sweep(node)
    for _ in range(200):
        if node.state != 'SWEEP':
            break
        node.advance(0.1)                       # el robot no responde
    assert node.state == 'STOPPED'
    assert node.sweep_targets and max(abs(t) for t in node.sweep_targets) == \
        pytest.approx(math.radians(6.0))


def test_sweep_does_not_start_off_axis_and_keeps_old_stop(make_node):
    node = make_node().ready(lateral=0.022)     # dentro de align_tol, fuera de sweep_tol
    node.on_timer()
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert all(msg.angular.z == 0.0 for msg in node.messages)


def test_sweep_does_not_start_far_from_contact(make_node):
    node = make_node().ready(distance=0.28)
    node.advance(12.1)                          # DOCK sin progreso -> WAIT_DOCK
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert all(msg.angular.z == 0.0 for msg in node.messages)


def test_bump_during_sweep_stops_without_backoff(make_node):
    node = make_node().ready()
    start_sweep(node)
    node.advance(0.05)
    msg = HazardDetectionVector()
    bump = HazardDetection()
    bump.type = HazardDetection.BUMP
    msg.detections = [bump]
    node.on_hazard(msg)
    assert node.last_command() == (0.0, 0.0)
    node.on_timer()
    assert node.state == 'STOPPED'
    assert node.n_retries == 0


@pytest.mark.parametrize('dry_run', [False, True])
def test_sweep_respects_global_limits_and_dry_run(make_node, dry_run):
    node = make_node(sweep_w=9.0, w_max=9.0, dry_run=dry_run).ready()
    start_sweep(node)
    node.advance(0.05)
    for msg in node.messages:
        assert msg.linear.x == 0.0
        assert abs(msg.angular.z) <= 0.50
        if dry_run:
            assert msg.angular.z == 0.0


@pytest.mark.parametrize('overrides', [{'sweep_max_deg': 12.0},
                                       {'sweep_step_deg': 8.0},
                                       {'sweep_hold_s': 0.0}])
def test_invalid_sweep_parameters_are_rejected(make_node, overrides):
    with pytest.raises(ValueError):
        make_node(**overrides)


def test_sweep_can_be_disabled(make_node):
    node = make_node(dock_sweep=False, **SEAT_GEOMETRY).ready()
    node.on_timer()
    node.advance(3.1)
    assert node.state == 'STOPPED'
    assert all(msg.angular.z == 0.0 for msg in node.messages)


def test_physical_defaults_match_rbot_calibration(make_node):
    # docs/pruebas/2026-11-02-calibracion-dock-manual.json: acoplado y cargando,
    # el centro queda a 0.2373 m de la pared. El tope debe quedar MAS ALLA de
    # esa posicion (si no, se para antes del contacto) y antes de tocar las
    # cajas (~0.225 m). El firmware declara min_speed=0.03 m/s.
    node = make_node()
    assert 0.225 < node.d_floor < 0.2373
    assert node.v_dock >= 0.03
    assert node.sweep and node.sweep_w * 0.1175 >= 0.03


def test_sweep_starts_when_resting_slightly_above_the_floor(make_node):
    # Corrida RBOT: con d_floor=0.231 el robot quedo quieto en ~0.237 m
    # (sesgo de ~7 mm medido en movimiento). Ahi debe barrer.
    node = make_node().ready()
    node.on_timer()
    assert node.state == 'WAIT_DOCK'
    node.filt.p[0] = 0.244
    node.advance(3.1)
    assert node.state == 'SWEEP'
