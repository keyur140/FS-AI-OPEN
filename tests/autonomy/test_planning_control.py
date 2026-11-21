# fsai-open: tests - local path, racing line, speed profile and path following
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import math

import numpy as np

from fsai.autonomy import config as C
from fsai.autonomy import planning as PL
from fsai.autonomy import track as T
from fsai.autonomy.control import PathFollower
from fsai.bridge.core import DriveCommand
from fsai.params import load

CFG = C.load()
VEH = load().vehicle


def test_local_path_runs_down_the_middle_of_a_straight():
    xs = np.arange(2.0, 25.0, 5.0)
    cones = np.array([[x, 1.5, T.BLUE] for x in xs] + [[x, -1.5, T.YELLOW] for x in xs], float)
    path, info = PL.local_path(cones, (0.0, 0.0, 0.0), CFG)
    assert info["mode"] == "delaunay"
    assert np.abs(path[path[:, 0] > 3, 1]).max() < 0.15


def test_local_path_turns_the_right_way_and_respects_colours():
    tr = T.trackdrive(2)
    path, info = PL.local_path(tr.cones, (0.0, 0.0, 0.0), CFG)
    d = np.min(np.hypot(path[:, None, 0] - tr.centreline[None, :, 0], path[:, None, 1] - tr.centreline[None, :, 1]), axis=1)
    assert info["mode"] == "delaunay" and d.max() < 0.5


def test_local_path_swapped_colours_is_penalised():
    """Test the test: the same cones with blue and yellow swapped must give a worse score."""
    xs = np.arange(2.0, 25.0, 5.0)
    good = np.array([[x, 1.5, T.BLUE] for x in xs] + [[x, -1.5, T.YELLOW] for x in xs], float)
    bad = good.copy()
    bad[:, 2] = 1 - bad[:, 2]
    _, gi = PL.local_path(good, (0, 0, 0), CFG)
    _, bi = PL.local_path(bad, (0, 0, 0), CFG)
    assert bi["score"] > gi["score"] + 20


def test_one_colour_only_falls_back_to_offset():
    xs = np.arange(2.0, 25.0, 4.0)
    cones = np.array([[x, 1.5, T.BLUE] for x in xs], float)
    path, info = PL.local_path(cones, (0, 0, 0), CFG)
    assert info["mode"] == "one-side offset" and np.abs(path[path[:, 0] > 3, 1]).max() < 0.2


def test_racing_line_stays_inside_the_cones_and_is_shorter_and_straighter():
    tr = T.trackdrive(3)
    ref, wl, wr = PL.centreline_from_map(tr.cones, tr.centreline)
    line = PL.racing_line(ref, wl, wr, CFG)
    from scipy.spatial import cKDTree
    d, _ = cKDTree(tr.cones[:, :2]).query(line)
    assert d.min() > CFG["planner"]["racing_margin_m"] - 0.25
    k_ref = np.abs(T.curvature(ref, True))
    k_line = np.abs(T.curvature(line, True))
    assert np.sum(k_line ** 2) * 0.5 < np.sum(k_ref ** 2) * 0.5 * 0.8
    v_ref = PL.speed_profile(ref, True, CFG, VEH)
    v_line = PL.speed_profile(line, True, CFG, VEH)
    assert PL.lap_time(line, v_line, True) < PL.lap_time(ref, v_ref, True)


def test_racing_line_is_fast_enough_to_build_on_the_car():
    """The real-time runs found the racing line took over a minute for a 300 m lap. It must take seconds."""
    import time
    tr = T.trackdrive(3)
    t0 = time.perf_counter()
    ref, wl, wr = PL.centreline_from_map(tr.cones, tr.centreline)
    PL.racing_line(ref, wl, wr, CFG)
    assert time.perf_counter() - t0 < 5.0


def test_racing_line_on_its_own_thread_does_not_stop_the_car():
    """Real-time mode: the line is built in the background while the car keeps driving lap 2."""
    from fsai.autonomy.sim import MISSION_OF, World
    w = World("trackdrive", 3)
    w.stack.plan_async = True
    w.bench.arm(MISSION_OF["trackdrive"])
    w.bench.go()
    while w.t < 120 and w.stack.phase != "race":
        w.step()
    assert w.stack.phase == "race" and w.stack.global_line is not None
    assert w.bench.vcu.state == 3 and len(w.ref.hit) == 0


def test_speed_profile_respects_lateral_and_braking_limits():
    tr = T.trackdrive(1)
    v = PL.speed_profile(tr.centreline, True, CFG, VEH)
    k = np.abs(T.curvature(tr.centreline, True))
    assert np.all(v ** 2 * k <= CFG["speed"]["a_lat_mps2"] * 1.001)
    assert v.max() <= CFG["speed"]["max_mps"] + 1e-9
    seg = np.hypot(*np.diff(np.vstack([tr.centreline, tr.centreline[:1]]), axis=0).T)
    dec = (v ** 2 - np.roll(v, -1) ** 2) / (2 * seg)
    assert dec.max() <= CFG["speed"]["a_brake_mps2"] * 1.001


def test_pure_pursuit_steady_circle_matches_bicycle_geometry():
    R = 12.0
    th = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    path = np.column_stack([R * np.sin(th), R - R * np.cos(th)])
    f = PathFollower(CFG, VEH.wheelbase_m)
    f.set_path(path, np.full(len(path), 5.0), True)
    for _ in range(20):
        cmd = f.command((0.0, 0.0, 0.0), 5.0)
    expected = math.degrees(math.atan(VEH.wheelbase_m / R))
    assert abs(cmd.steer_deg - expected) < 0.6
    assert cmd.speed_mps == 5.0


# ---------------------------------------------------------------- sim vs asphalt: actuator dead time
def test_emulator_steering_waits_for_its_dead_time():
    from fsai.params import load
    from fsai.vcu.lockstep import Bench
    for delay in (0.0, 0.12):
        b = Bench(load(vehicle__steer_delay_s=delay))
        assert b.arm(1) and b.go()
        t0 = b.t
        moved_at = None
        while b.t - t0 < 0.5:
            b.bridge.set_command(DriveCommand(speed_mps=0.0, steer_deg=10.0), b.t)
            b.step()
            if moved_at is None and abs(b.vcu.steer) > 0.1:
                moved_at = b.t - t0
        assert moved_at is not None and delay <= moved_at <= delay + 0.06, (delay, moved_at)


def test_latency_compensation_steers_for_the_future_pose():
    cfg = C.load(control__latency_comp_s=0.2, control__steer_smoothing=0.0)
    f = PathFollower(cfg, 1.53)
    xs = np.arange(0, 60, 0.5)
    f.set_path(np.column_stack([xs, np.zeros(len(xs))]), np.full(len(xs), 8.0), False)
    f.prev_steer = 10.0                         # turning left now -> in 0.2 s the car points left
    c = f.command((5.0, 0.0, 0.0), 8.0)
    assert c.steer_deg < 10.0                   # so it already steers back toward the path


def test_sensor_latency_shift_moves_a_late_scan_to_the_pose_of_now():
    """A cone seen 10 m ahead 0.1 s ago, while driving at 10 m/s straight, is 9 m ahead now."""
    from fsai.autonomy.perception import Cone
    from fsai.autonomy.stack import AutonomyStack
    st = AutonomyStack(C.load(), load())
    for k in range(30):
        st.on_odometry(k * 0.01, 10.0, 0.0)
    c = st._latency_shift([Cone(10.0, 0.0, 1, 1.0, "lidar")], stamp=0.19)
    assert abs(c[0].x - 9.0) < 0.05 and abs(c[0].y) < 1e-6


def test_settings_survive_pickling_for_the_planner_process():
    """The racing line is built in a separate process, which needs the settings pickled. On Python 3.10
    the attribute shortcut broke pickling and the car fell back to the slow local planner."""
    import pickle
    c = pickle.loads(pickle.dumps(CFG))
    assert c == CFG and c.speed["max_mps"] == CFG["speed"]["max_mps"]
