# fsai-open: the stage 2 event suite - every FS-AI dynamic event on seeded tracks, judged by a referee
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: a set of pretend competition days. Each run is one event on one generated track,
judged as the FS-AI 2026 rules judge it. A driving run passes only if the car finishes the right
number of laps, stops inside the allowed distance and says AS_FINISHED with no faults, touches no
cone, never leaves the track, the bridge never breaks a rule, and the map has no phantom cones.
The inspection and demo missions pass only if every step the rules list (IN2.2, IN2.3) happened.
A second set of runs makes the world worse on purpose (bad sensors, trackside clutter, less grip,
slower actuators) to see where it breaks."""
from __future__ import annotations

import multiprocessing as mp
import os

LAPS_REQUIRED = {"acceleration": 1, "skidpad": 4, "autocross": 1, "trackdrive": 10}
MISSIONS = ("inspection_a", "inspection_b", "demo")
SUITE = ([(m, 0) for m in MISSIONS] + [("demo", 1)]
         + [("acceleration", s) for s in range(3)] + [("skidpad", s) for s in range(3)]
         + [("autocross", s) for s in range(6)] + [("trackdrive", s) for s in (1, 3, 5)])
DETERMINISM = [("acceleration", 0), ("skidpad", 0), ("autocross", 0), ("trackdrive", 3), ("demo", 0)]
STRESS = {
    "baseline": {},
    "camera_colour_errors_x3": {"sensors__camera_colour_error_near": 0.03, "sensors__camera_colour_error_far": 0.36},
    "camera_phantoms_x5": {"sensors__camera_false_per_frame": 1.5},
    "lidar_dropout_30pct": {"sensors__lidar_dropout": 0.30},
    "lidar_noise_x3": {"sensors__lidar_noise_m": 0.06},
    "gyro_bias_1dps": {"sensors__imu_yaw_rate_bias_dps": 1.0},
    "wheel_speed_noise_x5": {"sensors__wheel_speed_noise_frac": 0.05},
    "camera_blind": {"sensors__camera_range_m": 0.0, "sensors__camera_false_per_frame": 0.0},
    # sim vs asphalt: what a real test day adds
    "trackside_clutter": {"sensors__clutter_level": 1.0},
    "cold_tyres_grip_0p8": {"vehicle__tyre_mu": 0.96},
    "bumpy_grip_15pct": {"vehicle__tyre_mu_variation": 0.15},
    "steer_gain_0p85": {"vehicle__steer_gain": 0.85},
    "actuator_delay_x3": {"vehicle__steer_delay_s": 0.12, "vehicle__drive_delay_s": 0.09, "vehicle__brake_delay_s": 0.12},
    "delay_x3_no_compensation": {"vehicle__steer_delay_s": 0.12, "vehicle__drive_delay_s": 0.09,
                                 "vehicle__brake_delay_s": 0.12, "control__latency_comp_s": 0.0},
    "real_world_mix": {"sensors__clutter_level": 1.0, "vehicle__tyre_mu": 1.08, "vehicle__tyre_mu_variation": 0.1,
                       "vehicle__steer_gain": 0.9, "vehicle__steer_delay_s": 0.08, "vehicle__drive_delay_s": 0.06,
                       "vehicle__brake_delay_s": 0.08},
}
PARAM_SECTIONS = ("vehicle", "vcu", "bridge")          # overrides for ads_dv.yaml; the rest go to autonomy.yaml
STRESS_SEEDS = (0, 1, 2)


def judge(r: dict) -> list[str]:
    """Reasons the run fails (empty list = pass)."""
    why = []
    if r["event"] in MISSIONS:
        mc = r.get("mission_checks") or {}
        why += [f"check failed: {k}" for k, v in mc.items() if v is False and k != "all_ok"]
        if r["bridge_violations"]:
            why.append(f"{len(r['bridge_violations'])} bridge rule breaks")
        if r["cones_hit"]:
            why.append(f"{r['cones_hit']} cones hit")
        return why
    if r["final_state"] != "AS_FINISHED":
        why.append(f"ended in {r['final_state']}")
    if r["faults"]:
        why.append(f"faults {r['faults']}")
    if r["bridge_violations"]:
        why.append(f"{len(r['bridge_violations'])} bridge rule breaks")
    if len(r["lap_times_referee_s"]) != LAPS_REQUIRED[r["event"]]:
        why.append(f"{len(r['lap_times_referee_s'])} timed laps, {LAPS_REQUIRED[r['event']]} required")
    if r["cones_hit"]:
        why.append(f"{r['cones_hit']} cones hit")
    if r["off_track_s"]:
        why.append(f"off track {r['off_track_s']} s")
    if r["map_false_cones"]:
        why.append(f"{r['map_false_cones']} phantom cones in the map")
    if r.get("unsafe_stop"):
        why.append(f"unsafe stop (stopped {r.get('stop_distance_m')} m after the line, "
                   f"{r.get('stop_allowed_m')} m allowed, in lane {r.get('stopped_in_lane')})")
    why += [f"DNF: {d}" for d in r.get("dnf_reasons", []) if not d.endswith("laps")]
    return why


def _run(job):
    event, seed, overrides, replay_dir = job
    from . import config as acfg
    from .sim import World
    from ..params import load as load_params
    pov = {k: v for k, v in overrides.items() if k.split("__")[0] in PARAM_SECTIONS}
    cov = {k: v for k, v in overrides.items() if k not in pov}
    w = World(event, seed, cfg=acfg.load(**cov), params=load_params(**pov))
    r = w.run(900)
    r["overrides"] = overrides
    r["fail_reasons"] = judge(r)
    r["pass"] = not r["fail_reasons"]
    if replay_dir:
        from .replay import write_replay
        from .. import AUTHOR
        p = write_replay(w, r, os.path.join(replay_dir, f"replay_{event}_seed{seed}.html"),
                         footer=f"fsai-open - {AUTHOR['name']} (ORCID {AUTHOR['orcid']}) - synthetic desk simulation")
        r["replay"] = os.path.basename(p)
    return r


def run_jobs(jobs, processes: int | None = None) -> list[dict]:
    processes = processes or min(4, os.cpu_count() or 1)
    ctx = mp.get_context("spawn")                     # same behaviour on Linux, macOS and Windows
    with ctx.Pool(processes) as pool:
        return pool.map(_run, jobs, chunksize=1)


def run_suite(replay_dir: str | None = None, processes: int | None = None) -> dict:
    first = {}
    jobs = []
    for ev, s in SUITE:
        rd = replay_dir if ev not in first and ev not in ("inspection_a", "inspection_b") else None
        first.setdefault(ev, s)
        jobs.append((ev, s, {}, rd))
    jobs += [(ev, s, {}, None) for ev, s in DETERMINISM]
    res = run_jobs(jobs, processes)
    main, again = res[:len(SUITE)], res[len(SUITE):]
    by_key = {(r["event"], r["seed"]): r for r in main}
    for r in again:
        m = by_key[(r["event"], r["seed"])]
        m["rerun_identical"] = m["trace_sha256"] == r["trace_sha256"]
    det_ok = all(by_key[k].get("rerun_identical") for k in DETERMINISM)
    return {"runs": main, "all_pass": all(r["pass"] for r in main) and det_ok, "determinism_ok": det_ok,
            "laps_required": LAPS_REQUIRED, "rules": "IMechE 2026 FS-AI Rules v1 (D4-D9, IN2.2, IN2.3, T4.2.2)"}


def run_stress(processes: int | None = None, event: str = "autocross") -> dict:
    jobs = [(event, s, ov, None) for name, ov in STRESS.items() for s in STRESS_SEEDS]
    res = run_jobs(jobs, processes)
    out = {}
    k = 0
    for name in STRESS:
        rows = res[k:k + len(STRESS_SEEDS)]
        k += len(STRESS_SEEDS)
        out[name] = {"overrides": STRESS[name], "event": event, "seeds": list(STRESS_SEEDS),
                     "passed": sum(r["pass"] for r in rows), "runs": len(rows),
                     "lap_times_s": [r["lap_times_referee_s"][0] if r["lap_times_referee_s"] else None for r in rows],
                     "official_times_s": [r.get("official_time_s") for r in rows],
                     "max_lateral_m": [r["max_lateral_m"] for r in rows],
                     "cones_hit": [r["cones_hit"] for r in rows],
                     "map_false_cones": [r["map_false_cones"] for r in rows],
                     "map_colour_accuracy": [r["map_colour_accuracy"] for r in rows],
                     "fail_reasons": [r["fail_reasons"] for r in rows]}
    return out
