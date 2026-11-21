# fsai-open: tests - whole events in the closed loop: stack -> bridge -> CAN bytes -> VCU emulator
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Trackdrive (10 laps, about 2 minutes of computing each) runs in run_all.py, not here."""
import pytest

from fsai.autonomy.sim import World
from fsai.bridge.core import DriveCommand


def _clean(r):
    assert r["final_state"] == "AS_FINISHED", r
    assert r["faults"] == [] and r["bridge_violations"] == []
    assert r["cones_hit"] == 0 and r["off_track_s"] == 0.0
    assert r["map_false_cones"] == 0


def test_autocross_end_of_lap_does_not_drag_the_map():
    """Seed 4: near the finish the filter's own pose uncertainty had grown to about 1.4 m, and a far cone was
    matched to a mapped cone 2.2 m away. That one match shifted the car and the whole map by 2 m (101
    phantom cones). Matches are now capped at slam.max_match_m."""
    r = World("autocross", 4).run(120)
    _clean(r)
    assert r["final_pose_error_m"] < 0.5, r["final_pose_error_m"]


def test_acceleration_finishes_inside_the_stop_area():
    r = World("acceleration", 0).run(60)
    _clean(r)
    assert len(r["lap_times_referee_s"]) == 1 and r["lap_times_referee_s"][0] < 12
    assert 0 < r["stop_distance_m"] <= 100 and r["stopped_in_lane"]     # FSAI26 D5.3.6
    assert r["official_time_s"] == r["lap_times_referee_s"][0] and not r["unsafe_stop"]


def test_skidpad_four_laps_then_exit():
    r = World("skidpad", 0).run(120)
    _clean(r)
    assert len(r["lap_times_referee_s"]) == 4
    assert r["laps_by_stack"] == 5                           # entry crossing + 4 laps through the centre
    laps = r["lap_times_referee_s"]                          # FSAI26 D4.4.1: mean of the timed laps 2 and 4
    assert r["official_time_s"] == pytest.approx((laps[1] + laps[3]) / 2, abs=1e-3)
    assert 0 < r["stop_distance_m"] <= 25 and r["stopped_in_lane"]      # FSAI26 D4.3.6


@pytest.mark.parametrize("seed", [5, 7])
def test_autocross_one_lap_and_a_correct_map(seed):
    r = World("autocross", seed).run(200)
    _clean(r)
    assert r["laps_by_stack"] == 1 and len(r["lap_times_referee_s"]) == 1
    assert r["map_colour_accuracy"] >= 0.98 and r["map_rmse_m"] < 0.2
    assert abs(r["lap_times_stack_s"][0] - r["lap_times_referee_s"][0]) < 1.0
    assert 0 < r["stop_distance_m"] <= 30 and not r["unsafe_stop"]      # FSAI26 D6.3.7


def test_same_seed_same_run_byte_for_byte():
    a = World("acceleration", 2).run(60)
    b = World("acceleration", 2).run(60)
    assert a["trace_sha256"] == b["trace_sha256"]
    c = World("acceleration", 3).run(60)
    assert c["trace_sha256"] != a["trace_sha256"]


def test_referee_sees_a_bad_driver():
    """Test the test: a stack that steers 6 degrees off must be caught (cones hit or off track)."""
    w = World("autocross", 5)
    real = w.stack.follow.command
    w.stack.follow.command = lambda pose, v: (lambda c: DriveCommand(
        speed_mps=c.speed_mps, steer_deg=c.steer_deg + 6.0, accel_mps2=c.accel_mps2, decel_mps2=c.decel_mps2))(real(pose, v))
    b = w.bench
    b.arm(3)
    b.go()
    while b.t < 40 and b.vcu.state == 3:
        w.step()
    r = w.result()
    assert r["cones_hit"] > 0 or r["off_track_s"] > 0


def test_res_emergency_stop_mid_run_is_clean():
    w = World("autocross", 5)
    b = w.bench
    b.arm(3)
    b.go()
    while b.t < 15:
        w.step()
    b.vcu.inputs.sdc_closed = False                          # marshal presses the RES
    for _ in range(600):
        w.step()
    r = w.result()
    assert r["final_state"] == "AS_EMERGENCY_BRAKE" and r["bridge_violations"] == []
    assert b.vcu.v == 0.0


def test_blind_camera_still_finishes_on_lidar_alone():
    """Camera fails completely: colours unknown, the planner must still find the gates by shape."""
    w = World("autocross", 5)
    w.camera.detect = lambda cones, pose, clutter=None: []
    r = w.run(200)
    assert r["final_state"] == "AS_FINISHED" and r["bridge_violations"] == []
    assert r["off_track_s"] == 0.0


# ---------------------------------------------------------------- FS-AI 2026 inspection and demo missions
@pytest.mark.parametrize("mission", ["inspection_a", "inspection_b", "demo"])
def test_inspection_and_demo_missions_meet_every_step_of_the_rules(mission):
    """FSAI26 IN2.2.2 (Static Inspection A and B) and IN2.3.2 (Autonomous Demo), checked on the truth."""
    r = World(mission, 0).run(90)
    assert r["bridge_violations"] == []
    assert r["mission_checks"]["all_ok"], r["mission_checks"]
    assert r["final_state"] == ("AS_FINISHED" if mission == "inspection_a" else "AS_EMERGENCY_BRAKE")


def test_lifted_car_does_not_move_during_static_inspection():
    w = World("inspection_a", 0)
    w.run(60)
    assert max(w.log["rpm"]) > 190 and abs(w.bench.vcu.x) < 1e-9 and abs(w.bench.vcu.y) < 1e-9


# ---------------------------------------------------------------- FS-AI 2026 T4.2.2: sensor loss -> EBS
def test_lost_lidar_and_camera_mid_run_requests_the_ebs():
    r = World("autocross", 5, sensor_cut_s=12.0).run(60)
    assert r["final_state"] == "AS_EMERGENCY_BRAKE" and r["ebs_requested_by_ai"]
    assert any("T4.2.2" in a for a in r["alerts"]), r["alerts"]
    assert r["bridge_violations"] == []


def test_lost_imu_alone_requests_the_ebs():
    w = World("autocross", 5)
    real = w.stack.mark
    w.stack.mark = lambda name, t: None if (name == "imu" and t > 10.0) else real(name, t)
    r = w.run(60)
    assert r["final_state"] == "AS_EMERGENCY_BRAKE"
    assert any("imu signal lost" in a for a in r["alerts"]), r["alerts"]


# ---------------------------------------------------------------- sim vs asphalt
def test_trackside_clutter_people_barriers_grass_and_shimmer():
    """Real tracks: barriers, people, photographers, long grass and heat shimmer must not become cones."""
    from fsai.autonomy import config as C
    r = World("autocross", 5, cfg=C.load(sensors__clutter_level=1.0)).run(200)
    assert r["final_state"] == "AS_FINISHED" and r["cones_hit"] == 0 and r["off_track_s"] == 0.0
    assert r["map_false_cones"] <= 2, r["map_false_cones"]


def test_slower_actuators_and_less_grip_still_finish_clean():
    from fsai.params import load
    p = load(vehicle__tyre_mu=1.0, vehicle__tyre_mu_variation=0.1, vehicle__steer_gain=0.9,
             vehicle__steer_delay_s=0.08, vehicle__brake_delay_s=0.08)
    r = World("autocross", 5, params=p).run(200)
    assert r["final_state"] == "AS_FINISHED" and r["cones_hit"] == 0 and r["off_track_s"] == 0.0


def test_late_scans_on_the_symmetric_skidpad_rings_stay_clean():
    """Real sensors arrive late and unevenly: each scan here is delivered 0.08 to 0.28 s after it was
    measured, with its measurement stamp. The ring layout repeats itself, so a scan used at the wrong
    pose would match the wrong cones; the stamps must be honoured."""
    r = World("skidpad", 0, sensor_latency_s=(0.08, 0.28)).run(120)
    assert r["final_state"] == "AS_FINISHED" and r["cones_hit"] == 0 and r["off_track_s"] == 0.0
    assert len(r["lap_times_referee_s"]) == 4


def test_a_sudden_pose_jump_on_the_frozen_map_is_recovered_not_lost():
    """A stall or slide can leave the pose 1 m off after the map is frozen. The stack must widen its
    search and find the map again instead of driving on lost (or stopping with the EBS)."""
    from fsai.autonomy.sim import MISSION_OF
    w = World("trackdrive", 3)
    w.bench.arm(MISSION_OF["trackdrive"])
    w.bench.go()
    while w.t < 200 and not (w.stack.phase == "race" and w.stack.lap >= 2):
        w.step()
    w.stack.slam.mu[0] += 0.8                         # knock the estimate sideways and turn it
    w.stack.slam.mu[1] += 0.6
    w.stack.slam.mu[2] += 0.06
    while w.t < 420 and w.bench.vcu.state not in (4, 5):
        w.step()
    r = w.result()
    # measured: without the widening the car is lost (EBS, 23 cones hit, 21.8 s off track); with it the
    # car finishes all laps, touching at most the one cone it is already heading for at the jump
    assert r["final_state"] == "AS_FINISHED" and r["cones_hit"] <= 1 and r["off_track_s"] == 0.0, \
        (r["final_state"], r["cones_hit"], r["off_track_s"])
    assert w.stack.n_relocalise >= 1
