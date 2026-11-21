#!/usr/bin/env python3
# fsai-open: one command - run every test and scenario, record everything, write reports and fingerprints
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Usage:
  python run_all.py                 # all tests + scenarios + reports + SHA256SUMS/MERKLE_ROOT
  python run_all.py --quick         # skip the ~25 s wall-clock (real-time) tests
  python run_all.py --ros-demo PATH # also record a ROS 2 bridge-demo verdict JSON (./run.sh bridge-demo)
  python run_all.py --ros-autonomy DIR  # also record ROS 2 driverless desk runs (result_*.json from ./run.sh demo)
  python run_all.py --skip-events   # skip the stage 2 driverless event suite (about 5 minutes on 4 cores)
Outputs: runs/<step>/..., runs/<step>/manifest.json, reports/RESULTS.md, reports/results.json,
SHA256SUMS.txt, MERKLE_ROOT.txt. Exit code 0 only if every test and every scenario passed.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from fsai import AI_ASSISTANCE, AUTHOR, MASTER_SEED, __version__  # noqa: E402
from fsai.params import DEFAULT_FILE  # noqa: E402
from fsai.provenance import environment, run_manifest, sha256_file, write_json, write_sha256sums  # noqa: E402

RUNS = ROOT / "runs"
REPORTS = ROOT / "reports"
DBC26 = ROOT / "third_party" / "dbc" / "ADS-DV_2026_CAN1.dbc"
DBC21 = ROOT / "third_party" / "dbc" / "ADSDV_2021_VCU_AI_interface_v2.dbc"


def step(title):
    print(f"\n=== {title}", flush=True)
    return time.time()


def run_tests(quick: bool) -> dict:
    out = RUNS / "tests"
    out.mkdir(parents=True, exist_ok=True)
    junit = out / "junit.xml"
    cmd = [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", f"--junitxml={junit}", "tests"]
    if quick:
        cmd += ["--ignore=tests/realtime"]
    t = time.time()
    # ROS 2 installs its own pytest plugins (launch_testing) that do not load under pytest 9; ours need none.
    env = {**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"}
    proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, env=env)
    (out / "pytest_output.txt").write_text(proc.stdout + proc.stderr, encoding="utf-8")
    print(proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else proc.stderr[-2000:])
    cases = []
    for tc in ET.parse(junit).getroot().iter("testcase"):
        outcome = "passed"
        for tag in ("failure", "error", "skipped"):
            node = tc.find(tag)
            if node is not None:
                outcome = {"failure": "failed", "error": "error", "skipped": "skipped"}[tag]
                reason = node.get("message", "")
                break
        else:
            reason = ""
        cases.append({"id": f"{tc.get('classname')}::{tc.get('name')}", "outcome": outcome,
                      "seconds": float(tc.get("time", 0)), "reason": reason[:300]})
    counts = {k: sum(c["outcome"] == k for c in cases) for k in ("passed", "failed", "error", "skipped")}
    summary = {"counts": counts, "total": len(cases), "exit_code": proc.returncode,
               "seconds": round(time.time() - t, 1), "cases": cases}
    write_json(out / "summary.json", summary)
    write_json(out / "manifest.json", run_manifest(
        "tests", {"dbc_2021": DBC21, "dbc_2026": DBC26, "params": DEFAULT_FILE},
        {"junit": junit, "summary": out / "summary.json", "output": out / "pytest_output.txt"},
        {"quick": quick, "seed": MASTER_SEED, "command": " ".join(cmd[1:])}))
    return summary


def run_scenarios() -> dict:
    from fsai.scenarios import run_all
    out = RUNS / "scenarios"
    results = run_all()
    again = run_all()                                # determinism: run twice, compare fingerprints
    for a, b in zip(results, again):
        a["rerun_identical"] = a["trace_sha256"] == b["trace_sha256"]
    summary = {"scenarios": results, "all_pass": all(r["pass"] and r["rerun_identical"] for r in results)}
    write_json(out / "results.json", summary)
    write_json(out / "manifest.json", run_manifest(
        "scenarios", {"params": DEFAULT_FILE}, {"results": out / "results.json"},
        {"seed": MASTER_SEED, "tick_s": 0.01}, notes="Deterministic lockstep: bridge + VCU emulator exchange "
        "encoded CAN bytes in simulated time. Each scenario run twice; fingerprints must match."))
    for r in results:
        print(f"  {'PASS' if r['pass'] and r['rerun_identical'] else 'FAIL'}  {r['scenario']:<22} {r['observed']}")
    return summary


def realtime_timing() -> dict:
    """Wall-clock loop timing on this machine (depends on hardware; reported, not a pass/fail claim)."""
    import threading
    import can
    from fsai.bridge.core import BridgeCore, DriveCommand
    from fsai.bridge.runner import LoopStats, bridge_step_fn, emulator_step_fn, periodic
    from fsai.params import load
    from fsai.vcu.emulator import VcuEmulator
    p = load()
    a = can.Bus(interface="virtual", channel="fsai-timing", receive_own_messages=False)
    b = can.Bus(interface="virtual", channel="fsai-timing", receive_own_messages=False)
    vcu, core, stop = VcuEmulator(p), BridgeCore(p), threading.Event()
    sv, sa = LoopStats(), LoopStats()
    th = [threading.Thread(target=periodic, args=(emulator_step_fn(vcu, a), stop, 0.01, sv)),
          threading.Thread(target=periodic, args=(bridge_step_fn(core, b), stop, 0.01, sa))]
    for t in th:
        t.start()
    vcu.inputs.asms = vcu.inputs.tsms = True
    vcu.inputs.ami = 1
    time.sleep(p.vcu.ready_min_s + 0.5)
    vcu.inputs.res_go = True
    t_end = time.monotonic() + 20
    while time.monotonic() < t_end:
        core.set_command(DriveCommand(speed_mps=8), time.monotonic())
        time.sleep(0.01)
    stop.set()
    for t in th:
        t.join()
    a.shutdown()
    b.shutdown()
    res = {"bridge_loop": sa.summary(), "vcu_loop": sv.summary(), "final_state": vcu.state,
           "faults": vars(vcu.faults), "budget_ms": 100, "machine": environment()}
    write_json(RUNS / "timing" / "results.json", res)
    write_json(RUNS / "timing" / "manifest.json", run_manifest(
        "timing", {"params": DEFAULT_FILE}, {"results": RUNS / "timing" / "results.json"},
        {"duration_s": 25.5, "bus": "python-can virtual"}, evidence_level="Offline-validated (this machine only)"))
    print(f"  bridge loop: {sa.summary()}")
    return res


def run_events(skip_stress: bool = False) -> dict:
    """Stage 2: every FS-AI dynamic event on seeded tracks, judged by the referee; then sensor stress runs."""
    from fsai.autonomy import events as E
    out = RUNS / "events"
    out.mkdir(parents=True, exist_ok=True)
    t = time.time()
    suite = E.run_suite(replay_dir=str(out))
    suite["seconds"] = round(time.time() - t, 1)
    write_json(out / "results.json", suite)
    for r in suite["runs"]:
        laps = r["lap_times_referee_s"]
        print(f"  {'PASS' if r['pass'] else 'FAIL'}  {r['event']:<12} seed {r['seed']}  laps {len(laps)}  "
              f"best {min(laps) if laps else '-'} s  cones hit {r['cones_hit']}  {'; '.join(r['fail_reasons'])}")
    print(f"  re-runs byte-identical: {suite['determinism_ok']}")
    stress = None
    if not skip_stress:
        t = time.time()
        stress = E.run_stress()
        write_json(out / "stress.json", {"event": "autocross", "seeds": list(E.STRESS_SEEDS), "conditions": stress,
                                         "seconds": round(time.time() - t, 1)})
        for name, d in stress.items():
            print(f"  stress {name:<26} {d['passed']}/{d['runs']} clean runs")
    from fsai.autonomy.config import AUTONOMY_FILE
    from fsai.autonomy.track import TRACKS_FILE
    outputs = {"results": out / "results.json"}
    if stress:
        outputs["stress"] = out / "stress.json"
    outputs.update({p.stem: p for p in sorted(out.glob("replay_*.html"))})
    write_json(out / "manifest.json", run_manifest(
        "events", {"params": DEFAULT_FILE, "autonomy": AUTONOMY_FILE, "tracks": TRACKS_FILE}, outputs,
        {"suite": [list(x) for x in E.SUITE], "determinism_reruns": [list(x) for x in E.DETERMINISM],
         "stress": E.STRESS, "stress_seeds": list(E.STRESS_SEEDS), "laps_required": E.LAPS_REQUIRED},
        notes="Closed loop in simulated time: seeded track -> pretend LiDAR/camera/IMU -> driving stack -> "
              "bridge -> encoded CAN bytes -> VCU emulator with tyre model. Referee sees the true track; "
              "the stack never does."))
    return {"suite": suite, "stress": stress}


def write_reports(tests, scen, timing, ros_demo, events=None, ros_auto=None):
    REPORTS.mkdir(exist_ok=True)
    ok = (tests["exit_code"] == 0 and scen["all_pass"]
          and (ros_demo is None or ros_demo.get("ok"))
          and (events is None or events["suite"]["all_pass"])
          and (not ros_auto or all(r.get("ok") for r in ros_auto)))
    results = {"version": __version__, "author": AUTHOR, "ai_assistance": AI_ASSISTANCE, "all_ok": ok,
               "tests": {k: tests[k] for k in ("counts", "total", "exit_code")},
               "scenarios": [{k: r[k] for k in ("scenario", "expected", "observed", "pass", "rerun_identical",
                                                 "trace_sha256")} for r in scen["scenarios"]],
               "timing": timing and {k: timing[k] for k in ("bridge_loop", "vcu_loop", "budget_ms")},
               "ros2_docker_demo": ros_demo and {k: ros_demo.get(k) for k in ("ok", "why", "distance_m",
                                                                                "max_speed_mps", "timeline")},
               "events": events and [{k: r.get(k) for k in ("event", "seed", "pass", "fail_reasons", "track_length_m",
                                                           "lap_times_referee_s", "cones_hit", "off_track_s",
                                                           "max_speed_mps", "map_true_cones", "map_false_cones",
                                                           "map_colour_accuracy", "map_rmse_m", "predicted_lap_s",
                                                           "official_time_s", "raw_time_s", "stop_distance_m",
                                                           "stop_allowed_m", "unsafe_stop", "oc_count", "dnf_reasons",
                                                           "mission_checks", "final_state",
                                                           "rerun_identical", "trace_sha256")}
                                     for r in events["suite"]["runs"]],
               "events_determinism_ok": events and events["suite"]["determinism_ok"],
               "stress": events and events["stress"] and {k: {x: v[x] for x in ("passed", "runs", "lap_times_s",
                                                                              "official_times_s", "cones_hit",
                                                                              "map_false_cones", "fail_reasons")}
                                                          for k, v in events["stress"].items()},
               "ros2_autonomy": ros_auto,
               "dbc_2026_sha256": sha256_file(DBC26) if DBC26.exists() else None,
               "environment": environment()}
    write_json(REPORTS / "results.json", results)
    c = tests["counts"]
    lines = [
        "# fsai-open: results of this build", "",
        f"**Author:** {AUTHOR['name']} · ORCID {AUTHOR['orcid']}  ",
        f"{AUTHOR['affiliations'][0]}; {AUTHOR['affiliations'][1]}  ",
        f"*{AUTHOR['note']}*  ",
        f"*AI assistance:* {AI_ASSISTANCE}", "",
        "Every number on this page is read from the files in `runs/`. Re-create it with `python run_all.py`.", "",
        "## In plain words", "",
        f"- **Overall: {'everything passed' if ok else 'SOMETHING FAILED - see below'}.**",
        f"- {c['passed']} automatic checks passed, {c['failed'] + c['error']} failed, {c['skipped']} skipped.",
        f"- {sum(r['pass'] for r in scen['scenarios'])} of {len(scen['scenarios'])} pretend test days ended exactly as "
        "the rules say they must, and each one gave the identical fingerprint when run a second time.",
    ]
    if timing:
        lines.append(f"- On this computer the 10 ms loop never took longer than {timing['bridge_loop']['max_period_ms']} ms; "
                     "the rules allow 50 ms before the car stops itself.")
    if ros_demo:
        lines.append(f"- The ROS 2 bridge demo in Docker {'finished cleanly' if ros_demo.get('ok') else 'FAILED'}: "
                     f"{ros_demo.get('why')}, {ros_demo.get('distance_m')} m travelled.")
    if events:
        ev = [r for r in events["suite"]["runs"] if not r.get("mission_checks")]
        mi = [r for r in events["suite"]["runs"] if r.get("mission_checks")]
        lines.append(f"- **Driving software:** {sum(r['pass'] for r in ev)} of {len(ev)} pretend competition runs "
                     "(acceleration, skidpad, autocross, 10-lap trackdrive on generated tracks) were clean as the "
                     "FS-AI 2026 rules judge them: right number of laps, stopped inside the allowed distance, "
                     "AS Finished, no cone touched, never off track, no rule broken, no phantom cones in the map. "
                     f"Re-runs gave identical fingerprints: {'yes' if events['suite']['determinism_ok'] else 'NO'}.")
        lines.append(f"- **Inspection missions:** {sum(r['pass'] for r in mi)} of {len(mi)} runs of Static Inspection A, "
                     "Static Inspection B and the Autonomous Demo did every step the rules list (IN2.2.2, IN2.3.2).")
    if ros_auto:
        lines.append(f"- **ROS 2 in Docker, real time:** {sum(bool(r.get('ok')) for r in ros_auto)} of {len(ros_auto)} "
                     "driverless desk runs finished cleanly with every part as a separate process.")
    lines += ["- What this does **not** show: anything about the real car. All of this is software against a "
              "software copy of the car's safety rules. The real check is the team's first powered, "
              "wheels-off session (see docs/TEAM_SETUP_GUIDE.md).", "",
              "## Evidence level", "", "**Synthetic-validated** (desk tests against a specification emulator). "
              "Not yet run on an ADS-DV.", "",
              "## Scenarios", "", "| Scenario | Expected | Observed | Pass | Re-run identical | Fingerprint |",
              "|---|---|---|---|---|---|"]
    for r in scen["scenarios"]:
        lines.append(f"| {r['scenario']} | {r['expected']} | {r['observed']} | {'yes' if r['pass'] else 'NO'} | "
                     f"{'yes' if r['rerun_identical'] else 'NO'} | `{r['trace_sha256'][:16]}…` |")
    if events:
        lines += ["", "## Inspection missions (FS-AI 2026 rules IN2.2.2 and IN2.3.2)", "",
                  "| Mission | Seed | End state | Checks | Pass |", "|---|---|---|---|---|"]
        for r in events["suite"]["runs"]:
            mc = r.get("mission_checks")
            if not mc:
                continue
            shown = "; ".join(f"{k.replace('_', ' ')}: {v}" for k, v in mc.items() if k != "all_ok")
            lines.append(f"| {r['event']} | {r['seed']} | {r['final_state']} | {shown} | "
                         f"{'yes' if r['pass'] else 'NO: ' + '; '.join(r['fail_reasons'])} |")
        lines += ["", "## Driverless events (stage 2, judged by the FS-AI 2026 rules D4-D9)", "",
                  "| Event | Seed | Track [m] | Laps | Lap times [s] | Official time [s] | Stop after line [m] (allowed) | "
                  "Cones hit | Off track [s] | Max speed [m/s] | Map cones (false) | Colour acc. | Pass | Re-run identical |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in events["suite"]["runs"]:
            if r.get("mission_checks"):
                continue
            lt = r["lap_times_referee_s"]
            shown = ", ".join(f"{x:.2f}" for x in lt) if len(lt) <= 4 else f"{lt[0]:.2f}, then {min(lt[1:]):.2f}-{max(lt[1:]):.2f}"
            lines.append(f"| {r['event']} | {r['seed']} | {r['track_length_m']} | {len(lt)} | {shown} | "
                         f"{r.get('official_time_s')} | {r.get('stop_distance_m')} ({r.get('stop_allowed_m')}) | {r['cones_hit']} | "
                         f"{r['off_track_s']} | {r['max_speed_mps']} | {r['map_true_cones']} ({r['map_false_cones']}) | "
                         f"{r['map_colour_accuracy']} | {'yes' if r['pass'] else 'NO: ' + '; '.join(r['fail_reasons'])} | "
                         f"{ {True: 'yes', False: 'NO', None: '-'}[r.get('rerun_identical')] } |")
        if events["stress"]:
            lines += ["", "### Stress (autocross, seeds 0-2): bad sensors and sim-vs-asphalt effects. Where does it break?", "",
                      "| Condition | Changed settings | Clean runs | Official times [s] | Cones hit | Phantom map cones | Why not clean |",
                      "|---|---|---|---|---|---|---|"]
            for name, d in events["stress"].items():
                ov = ", ".join(f"{k.split('__')[1]}={v}" for k, v in d["overrides"].items()) or "-"
                why = "; ".join(sorted({w for f in d["fail_reasons"] for w in f})) or "-"
                lines.append(f"| {name} | {ov} | {d['passed']}/{d['runs']} | {d['official_times_s']} | {d['cones_hit']} | "
                             f"{d['map_false_cones']} | {why} |")
        lines += ["", "Replays: open `runs/events/replay_*.html` in any browser (one per event, seed shown)."]
    if events and events["stress"]:
        lines += ["", "A stress run is \"not clean\" for any one fault, including a single phantom cone left in the map "
                  "(a piece of trackside clutter that looked like a cone often enough). Cones hit and time off track "
                  "are the columns that matter for driving."]
    if ros_auto:
        lines += ["", "## ROS 2 in Docker, real time (every part a separate process, sim CAN bus)", "",
                  "| Run | End state | Laps | Lap times [s] | Cones hit | Off track [s] | Max off line [m] | Clean |",
                  "|---|---|---|---|---|---|---|---|"]
        for r in ros_auto:
            lt = r.get("lap_times_referee_s") or []
            shown = ", ".join(f"{x:.2f}" for x in lt) if len(lt) <= 4 else f"{lt[0]:.2f}, then {min(lt[1:]):.2f}-{max(lt[1:]):.2f}"
            name = (r.get("run") or f"{r.get('event')}_{r.get('seed')}").replace("_", " ")
            lines.append(f"| {name} | {r.get('final_state')} | {len(lt)} | {shown or '-'} | {r.get('cones_hit')} | "
                         f"{r.get('off_track_s')} | {r.get('max_lateral_m')} | {'yes' if r.get('ok') else 'NO'} |")
        lines += ["", "Inspection B and the Autonomous Demo end in AS Emergency on purpose: the rules ask for an EBS "
                  "test at the end of each (IN2.3.2)."]
    exp = RUNS / "exploratory_real" / "results.json"
    if exp.exists():
        e = json.loads(exp.read_text())
        lines += ["", "## Exploratory run on real LiDAR scans (stage 3)", "",
                  f"Our LiDAR cone detector, shipped settings, no tuning, on {e['scene']['frames']} labelled frames "
                  f"of one real Formula Student scene ({e['scene']['source']}, `{e['scene']['file']}`, SHA-256 "
                  f"`{(e['scene']['sha256'] or '')[:16]}…`), and the same measure on fsai-open's own simulated scans. "
                  "Front view (+-80 deg). Data card: `data_cards/real_lidar_conescenes.md`.", "",
                  "| Range [m] | Real: labelled | Real: found | Real: false | Real: error [m] | Sim: labelled | Sim: found | Sim: false | Sim: error [m] |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for a_, b_ in zip(e["real"]["front_80deg"]["by_range"], e["synthetic"]["front_80deg"]["by_range"]):
            lines.append(f"| {a_['range_m']} | {a_['labelled_cones']} | {a_['found']} ({a_['recall']}) | {a_['false_cones']} | "
                         f"{a_['mean_position_error_m']} | {b_['labelled_cones']} | {b_['found']} ({b_['recall']}) | "
                         f"{b_['false_cones']} | {b_['mean_position_error_m']} |")
        ra, sy = e["real"]["front_80deg"]["summary"], e["synthetic"]["front_80deg"]["summary"]
        lines += ["", f"Up to {e['method']['headline_max_range_m']:.0f} m: real recall {ra['recall']}, precision "
                  f"{ra['precision']}, position error {ra['mean_position_error_m']} m; simulated recall {sy['recall']}, "
                  f"precision {sy['precision']}, error {sy['mean_position_error_m']} m. Exploratory: one scene, one sensor."]
    lines += ["", "## Tests by group", "", "| Group | Passed | Failed | Skipped |", "|---|---|---|---|"]
    groups = {}
    for case in tests["cases"]:
        g = case["id"].split("::")[0].replace("tests.", "")
        d = groups.setdefault(g, {"passed": 0, "failed": 0, "skipped": 0})
        d["failed" if case["outcome"] in ("failed", "error") else case["outcome"]] += 1
    for g, d in sorted(groups.items()):
        lines.append(f"| `{g}` | {d['passed']} | {d['failed']} | {d['skipped']} |")
    bad = [x for x in tests["cases"] if x["outcome"] != "passed"]
    if bad:
        lines += ["", "## Not passed", ""] + [f"- `{x['id']}`: {x['outcome']} {x['reason']}" for x in bad]
    lines += ["", "## Where each number comes from", "",
              "| Number | File |", "|---|---|",
              "| Test counts | `runs/tests/summary.json`, `runs/tests/junit.xml` |",
              "| Scenarios and fingerprints | `runs/scenarios/results.json` |",
              "| Loop timing | `runs/timing/results.json` |",
              "| ROS 2 Docker bridge demo | `runs/ros2_demo/result.json` |",
              "| Driverless events, stress runs, replays | `runs/events/results.json`, `runs/events/stress.json`, `runs/events/replay_*.html` |",
              "| ROS 2 Docker driverless runs | `runs/ros2_autonomy/result_*.json` |",
              "| Exploratory real-data run | `runs/exploratory_real/results.json` |",
              "| Every file's SHA-256 | `SHA256SUMS.txt`, root in `MERKLE_ROOT.txt` |", ""]
    (REPORTS / "RESULTS.md").write_text("\n".join(lines), encoding="utf-8")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--ros-demo", type=Path, default=None)
    ap.add_argument("--ros-autonomy", type=Path, default=None)
    ap.add_argument("--skip-events", action="store_true")
    ap.add_argument("--skip-stress", action="store_true")
    ap.add_argument("--in-container", action="store_true")
    ap.add_argument("--reports-only", action="store_true",
                    help="rewrite reports and fingerprints from the results already in runs/ (no re-running)")
    ap.add_argument("--no-hash", action="store_true", help="do not rewrite SHA256SUMS.txt / MERKLE_ROOT.txt")
    a = ap.parse_args()
    print(f"fsai-open {__version__} - {AUTHOR['name']}")
    if not DBC26.exists():
        subprocess.run([sys.executable, str(ROOT / "tools" / "fetch_dbc.py")])
    if a.reports_only:
        rd = lambda p: json.loads((RUNS / p).read_text()) if (RUNS / p).exists() else None
        st = rd("events/stress.json")
        events = {"suite": rd("events/results.json"), "stress": st and st["conditions"]}
        ros_auto = [{**json.loads(f.read_text()), "run": f.stem[len("result_"):]}
                    for f in sorted((RUNS / "ros2_autonomy").glob("result_*.json"))] or None
        ok = write_reports(rd("tests/summary.json"), rd("scenarios/results.json"), rd("timing/results.json"),
                           rd("ros2_demo/result.json"), events if events["suite"] else None, ros_auto)
        if not a.no_hash:
            print(f"  Merkle root: {write_sha256sums(ROOT)}")
        print("\nALL OK" if ok else "\nSOMETHING FAILED - see reports/RESULTS.md")
        return 0 if ok else 1
    step("1/5 tests")
    tests = run_tests(a.quick)
    step("2/5 scenarios (each run twice)")
    scen = run_scenarios()
    timing = None
    if not a.quick:
        step("3/5 real-time loop timing (25 s)")
        timing = realtime_timing()
    ros_demo = None
    if a.ros_demo and a.ros_demo.exists():
        ros_demo = json.loads(a.ros_demo.read_text())
        dest = RUNS / "ros2_demo" / "result.json"
        write_json(dest, ros_demo)
        write_json(RUNS / "ros2_demo" / "manifest.json", run_manifest(
            "ros2_demo", {"params": DEFAULT_FILE}, {"result": dest},
            {"launch": "fsai_ros demo.launch.py", "bus": "sim (udp_multicast)"},
            notes="Produced inside the fsai-open Docker image by ./run.sh demo"))
    else:
        old = RUNS / "ros2_demo" / "result.json"
        if old.exists():
            ros_demo = json.loads(old.read_text())
    events = None
    if not a.skip_events:
        step("4/5 driverless events (stage 2), then sensor stress runs")
        events = run_events(skip_stress=a.skip_stress or a.quick)
    ros_auto = None
    src = a.ros_autonomy if a.ros_autonomy and a.ros_autonomy.exists() else RUNS / "ros2_autonomy"
    files = sorted(src.glob("result_*.json")) if src.exists() else []
    if files:
        ros_auto = [{**json.loads(f.read_text()), "run": f.stem[len("result_"):]} for f in files]
        dest = RUNS / "ros2_autonomy"
        dest.mkdir(parents=True, exist_ok=True)
        for f in files:
            if f.parent.resolve() != dest.resolve():
                (dest / f.name).write_bytes(f.read_bytes())
                rep = f.with_name(f.name.replace("result_", "replay_").replace(".json", ".html"))
                if rep.exists():
                    (dest / rep.name).write_bytes(rep.read_bytes())
        write_json(dest / "manifest.json", run_manifest(
            "ros2_autonomy", {"params": DEFAULT_FILE}, {p.name: p for p in sorted(dest.glob("re*_*.*"))},
            {"launch": "fsai_ros autonomy_sim.launch.py", "bus": "sim (udp_multicast)", "clock": "real time"},
            notes="Produced inside the fsai-open Docker image by ./run.sh demo <event> <seed>"))
    step("5/5 reports and fingerprints")
    ok = write_reports(tests, scen, timing, ros_demo, events, ros_auto)
    write_json(RUNS / "run_all" / "manifest.json", run_manifest(
        "run_all", {"params": DEFAULT_FILE}, {"results": REPORTS / "results.json", "report": REPORTS / "RESULTS.md"},
        {"quick": a.quick, "seed": MASTER_SEED}))
    if not a.no_hash:
        root = write_sha256sums(ROOT)
        print(f"  Merkle root: {root}")
    print("\nALL OK" if ok else "\nSOMETHING FAILED - see reports/RESULTS.md")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
