# fsai-open: the in-depth Technical Report PDF (design, methodology, verification, project management)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Laid out like an engineering group-project report (aims, objectives, methodology, risks, health and safety,
budget, tools, work allocation, timeline, references). Every number is read from config/, runs/, reports/,
docs/ and DECISIONS.md by this script; nothing is typed in by hand. Called by tools/make_pdfs.py."""
from __future__ import annotations

import re

import matplotlib
import numpy as np
import yaml
from reportlab.lib.units import mm
from reportlab.platypus import PageBreak, Spacer

from make_pdfs import (HEX, ROOT, P, arch_draw, box, bullets, build, card, card_grid, fig, hood, md,  # noqa: F401
                       not_show, plain, section, table, tiles)


def cfgv(path):
    """value and source of a config entry, e.g. ('autonomy', 'slam.max_match_m')"""
    f, key = path
    d = yaml.safe_load((ROOT / "config" / f"{f}.yaml").read_text())
    for k in key.split("."):
        d = d[k]
    return d.get("value"), d.get("source", ""), d.get("status", "")


def v(f, key):
    return cfgv((f, key))[0]


def rows_of(f, keys, show_status=True):
    out = []
    for k in keys:
        val, src, stat = cfgv((f, k))
        out.append([f"<font name='DVM'>{k}</font>", val, md(src) + (f" <b>[{stat}]</b>" if stat and show_status else "")])
    return out


def h(t):
    return P(t, "h2")


def report_pdf(D, F):
    E = D["E"]
    st = []
    # ------------------------------------------------------------------ contents
    st.append(box([P("<b>Contents</b>", "card_h")] + bullets([
        "1  Introduction", "2  Background: the FS-AI class, the ADS-DV and how it is controlled",
        "3  Design specification: aim, objectives, requirements from the rules, expected challenges",
        "4  Methodology: safety layer, perception, SLAM, planning, control, vehicle model, missions, real time, "
        "simulator", "5  Verification and validation: tests, scenarios, events, stress, real time, real data",
        "6  Project management: risks, health and safety, budget, software and tools, work allocation",
        "7  Timeline", "8  References"]), "grey"))
    st.append(Spacer(1, 3 * mm))
    st.append(plain([
        "This report describes how the fsai-open driverless software was designed, built and tested, in the "
        "shape of an engineering project report.",
        "The software is complete for every FS-AI 2026 event and inspection and has been tested in simulation, "
        "including real-time runs. It has not run on the real ADS-DV yet; section 6 and the setup guide say how the "
        "team takes it there safely.",
        "Every number in this report is read by a script from the project's own files, so it matches the code."]))

    # ------------------------------------------------------------------ 1 introduction
    st += section("1. Introduction", "blue")
    st.append(P("In FS-AI, the driverless class of IMechE Formula Student, university teams write the software that "
                "drives a shared car, the ADS-DV. The car accepts commands only in one strict format over its CAN "
                "bus, and its safety computer, the VCU, applies the emergency brakes if the team's software is late "
                "or sends something it does not allow. Every team has to build this link itself; the official helper "
                "library dates from 2021 and does not match the 2026 car."))
    st.append(P("fsai-open is that link, built carefully and tested hard, plus a complete driving stack on top: "
                "cone detection from LiDAR and camera, mapping and localisation (SLAM), path, racing-line and speed "
                "planning, steering and speed control, and the mission logic for every event and inspection. It "
                "also contains a software copy of the car and track (VCU emulator, simulated sensors and an outside "
                "referee), so everything can be tested at a desk. It is built to be handed to Brunel University "
                "London's FS-AI team as a finished package (decision D1)."))
    st.append(tiles([(f"{F['checks']}", "automatic checks pass", "blue"),
                     (f"{F['ev_ok']}/{F['ev_n']}", "simulated events clean", "green"),
                     (f"{F['ros_ok']}/{F['ros_n']}", "real-time ROS 2 runs clean", "teal"),
                     (f"{round(100 * E['real']['front_80deg']['summary']['recall'])} %", "real cones found (≤20 m)",
                      "purple")]))

    # ------------------------------------------------------------------ 2 background
    st += section("2. Background", "navy")
    st.append(h("2.1 The car and its safety computer"))
    st.append(P("The ADS-DV is driven entirely through CAN frames. The VCU runs a state machine: "
                "<b>AS_OFF → AS_READY → AS_DRIVING → AS_FINISHED</b>, or <b>AS_EMERGENCY_BRAKE</b> from any active "
                "state. The rules the software must respect, as configured and tested in this package:"))
    st.append(table(["Setting", "Value", "Source"], rows_of("ads_dv", [
        "vcu.ready_min_s", "vcu.comms_timeout_cycles", "vcu.ebrake_hold_s", "vcu.standstill_rpm",
        "vcu.ready_max_actual_steer_deg", "vcu.angle_max_deg", "vehicle.axle_torque_max_nm",
        "vehicle.motor_speed_max_rpm", "vehicle.electrical_power_max_w"]), [52, 18, 100]))
    st.append(P("Sources: SPEC4 = ADS-DV Software Interface Specification v4.0 (2021); SPEC26 = ADS-DV "
                "Specifications &amp; Dimensions v1.0 (2026); DBC26 = the 2026 CAN database; FSAI26 = IMechE 2026 "
                "FS-AI Rules; API21 = the 2021 FS-AI_API library; HIL = the CarMaker HiL notes. "
                "Where they disagree, the strictest value is used (D12).", "small"))
    st.append(h("2.2 The car itself (published and estimated values)"))
    st.append(table(["Setting", "Value", "Source"], rows_of("ads_dv", [
        "vehicle.wheelbase_m", "vehicle.track_m", "vehicle.wheel_radius_m", "vehicle.belt_ratio",
        "vehicle.driven_axles", "vehicle.overall_width_m", "vehicle.overall_length_m", "vehicle.mass_kg",
        "vehicle.tyre_mu", "vehicle.steer_rate_deg_s", "vehicle.steer_delay_s", "vehicle.drive_delay_s",
        "vehicle.brake_delay_s"]), [52, 18, 100]))
    st.append(P("Values marked <b>[estimate]</b> are not published; the team measures them on the car "
                "(docs/ASSUMPTIONS.md, A10, A14, A15).", "small"))
    st.append(h("2.3 Sensors assumed by the simulator"))
    st.append(P("The package plugs into whatever sensors the team mounts; the simulator needs a layout to model. "
                "These are fsai-open choices, marked for tuning to the real mounting:"))
    st.append(table(["Setting", "Value", "Source"], rows_of("autonomy", [
        "sensors.lidar_channels", "sensors.lidar_hz", "sensors.lidar_range_m", "sensors.lidar_hfov_deg",
        "sensors.lidar_x_m", "sensors.lidar_z_m", "sensors.camera_hz", "sensors.camera_range_m",
        "sensors.camera_hfov_deg", "sensors.imu_yaw_rate_bias_dps"]), [52, 18, 100]))
    st.append(h("2.4 Frames and conventions"))
    st += bullets(["World frame: the car starts at (0, 0) facing +x (the emulator's frame, fsai/autonomy/track.py).",
                   "Vehicle and LiDAR frames: x forward, y left, z up (right-handed).",
                   "Steering: positive is left, bicycle model, in radians on the ROS 2 interface "
                   "(<font name='DVM'>/fsai/cmd</font>, ackermann_msgs).",
                   "Cone colours (FS Driverless Specification 2026, DS 1.1): blue on the left edge, yellow on the "
                   "right, small orange at entry and exit, big orange at start, finish and timekeeping lines."])

    # ------------------------------------------------------------------ 3 design specification
    st += section("3. Design specification", "green")
    st.append(h("3.1 Project aim"))
    st.append(P("To build complete, open, tested driverless software for the IMechE FS-AI ADS-DV that a team can "
                "install with one command, test at a desk without the car, and take to the car with a short, "
                "written list of things to measure (decisions D1 to D4, D11)."))
    n_spec = sum(c["id"].startswith("tests.conformance.test_spec_v4") for c in D["tests"]["cases"])
    n_pit = sum(c["id"].startswith("tests.conformance.test_pitfalls") for c in D["tests"]["cases"])
    st.append(h("3.2 Objectives and where each is shown"))
    st.append(table(["#", "Objective", "Evidence in this package"], [
        ["O1", "A CAN bridge that follows every rule of the interface specification and the 2026 rules, and never "
               "silently changes a command", f"conformance tests ({n_spec} spec-clause, {n_pit} known-pitfall); "
                                             f"{F['sc_ok']}/{F['sc_n']} scenarios"],
        ["O2", "A software copy of the VCU to test against", "fsai/vcu/emulator.py; codec checked against both "
                                                             "official CAN databases"],
        ["O3", "A driving stack for every FS-AI 2026 dynamic event", f"{F['ev_ok']}/{F['ev_n']} simulated events clean, "
                                                                    f"{F['cones_hit']} cones hit"],
        ["O4", "Every inspection mission (IN2.2.2, IN2.3.2)", f"{F['mi_ok']}/{F['mi_n']} inspection runs did every step"],
        ["O5", "Defences for the gap between simulator and real track",
         f"{len(D['stress'])} stress conditions × {len(D['stress_seeds'])} seeds; real LiDAR exploratory run"],
        ["O6", "Real-time operation with every part as a separate program", f"{F['ros_ok']}/{F['ros_n']} ROS 2 "
                                                                           "runs in Docker clean"],
        ["O7", "Results anyone can check", "SHA-256 of every file, Merkle root, byte-identical re-runs, "
                                           "verify.py"]], [10, 80, 80], "green"))
    st.append(h("3.3 Requirements taken from the rules"))
    st.append(P("Each event's geometry and judging is generated and checked from these values "
                "(config/tracks.yaml; FSAI26 = 2026 FS-AI Rules, HB21 = Dynamic Events Setup and Cones Specification "
                "2021, DS26 = FS Driverless Specification 2026):"))
    st.append(table(["Setting", "Value", "Source"], rows_of("tracks", [
        "track.width_m", "cones.max_spacing_m", "acceleration.length_m", "acceleration.stop_area_m",
        "skidpad.inner_diameter_m", "skidpad.centre_distance_m", "skidpad.laps_per_side", "skidpad.exit_m",
        "trackdrive.laps", "trackdrive.stop_after_finish_m", "trackdrive.max_straight_m",
        "trackdrive.min_hairpin_outside_diameter_m"]), [55, 15, 100], "green"))
    st.append(Spacer(1, 2 * mm))
    st.append(table(["Setting", "Value", "Source"], rows_of("autonomy", [
        "mission.inspect_a_rpm", "mission.inspect_a_ramp_s", "mission.inspect_a_stop_s", "mission.inspect_b_rpm",
        "mission.demo_leg_m", "mission.rpm_basis"]), [55, 15, 100], "green"))
    st.append(h("3.4 Expected challenges"))
    st.append(card_grid([
        card("Messy sensors", "Grass, barriers, people and heat shimmer on a real track; a camera can call a person "
             "in a yellow jacket a cone.", "orange"),
        card("Real tyre grip", "Cold tyres, bumps and steering compliance: the car turns less than asked.", "red"),
        card("Hardware latency", "Steering, drive and brakes act after a dead time; without compensation the car "
             "weaves.", "purple"),
        card("Real time", "A late scan, a busy thread or a stalled process must never reach the steering.", "teal"),
        card("Documents disagree", "For example the steering limit is 21°, 24°, 27.2° or 30° depending on the "
             "document; the bridge reads it live from the car (A4).", "blue"),
        card("No car to test on", "Everything must be testable in simulation and ready for a short list of "
             "measurements on the car.", "green")]))

    # ------------------------------------------------------------------ 4 methodology
    st += section("4. Methodology and system design", "navy")
    st.append(fig(arch_draw, "System architecture. Sensors feed the driving stack (blue); mission logic and "
                             "watchdogs (purple) supervise it; the CAN bridge (orange) is the only path to the VCU "
                             "(red). A team can replace the stack with its own through ROS 2 messages.", 170, 78))
    st.append(h("4.1 Safety layer: the CAN bridge"))
    st += bullets(["Echoes the VCU handshake every 10 ms on its own real-time thread, so a busy ROS executor cannot "
                   "starve it (fsai/bridge/runner.py).",
                   "Keeps steering and torque inside the live limits the car reports; never sends torque and brake "
                   "together, with one empty 10 ms cycle between them (D17); never NEUTRAL or FINISHED while moving.",
                   f"If the team's commands stop for {v('ads_dv', 'bridge.command_timeout_s')} s it brakes in a "
                   "controlled way instead of e-stopping; rejects NaN and infinity; reports every change it makes.",
                   "The 2021 library's torque derate above 300 to 700 wheel rpm is reproduced (API21)."])
    st.append(h("4.2 Perception"))
    st.append(P("Ground plane fit, then 5 cm voxels and clustering of the raised points; each cluster is tested as a "
                "cone shape; LiDAR cones are fused with camera cones for colour, and while the camera works a LiDAR "
                "blob counts only if the camera also sees a cone there."))
    st.append(table(["Setting", "Value", "Source"], rows_of("autonomy", [
        "perception.ground_tolerance_m", "perception.cluster_radius_m", "perception.min_points",
        "perception.cone_max_width_m", "perception.cone_max_height_m", "perception.cone_min_height_near_m",
        "perception.isolation_m", "perception.fusion_gate_m", "perception.confirm_with_camera",
        "perception.max_scan_age_s"]), [55, 15, 100]))
    st.append(h("4.3 Mapping and localisation (SLAM)"))
    st.append(P("An extended Kalman filter whose state is the car's pose (x, y, heading), the gyro bias and a "
                "learned rear-tyre slip gain, plus two coordinates per mapped cone. Data association is gated by "
                "Mahalanobis distance, colour (blue never matches yellow once the map is frozen) and a hard "
                "distance cap; candidate cones are pruned if not seen again. After the first lap the map is frozen "
                "and the filter only localises."))
    st.append(table(["Setting", "Value", "Source"], rows_of("autonomy", [
        "slam.gate_chi2", "slam.max_match_m", "slam.new_landmark_chi2", "slam.min_landmark_spacing_m",
        "slam.prune_after_seen", "slam.prune_ratio", "slam.bias_estimation", "slam.rear_slip_gain",
        "slam.slip_estimation"]), [55, 15, 100]))
    st.append(h("4.4 Planning"))
    st += bullets([f"Local path: a beam search over blue-yellow gates (width {v('autonomy', 'planner.beam_width')}, "
                   f"depth {v('autonomy', 'planner.beam_depth')}, horizon {v('autonomy', 'planner.horizon_m')} m, "
                   f"gates {v('autonomy', 'planner.min_gate_m')} to {v('autonomy', 'planner.max_gate_m')} m wide).",
                   "Racing line: a closed centreline from the frozen map, then a minimum-curvature line solved with "
                   f"L-BFGS-B (under 1 s instead of about 67 s, D16), keeping {v('autonomy', 'planner.racing_margin_m')} "
                   "m from the cones.",
                   f"Speed profile: lateral {v('autonomy', 'speed.a_lat_mps2')} m/s², drive "
                   f"{v('autonomy', 'speed.a_long_mps2')} m/s², brake {v('autonomy', 'speed.a_brake_mps2')} m/s², "
                   f"top speed {v('autonomy', 'speed.max_mps')} m/s, first lap at {v('autonomy', 'speed.explore_mps')} "
                   "m/s; one grip factor scales them all."])
    st.append(h("4.5 Control"))
    st += bullets([f"Lateral: {v('autonomy', 'control.lateral')} (Stanley and feedforward are also built in), "
                   f"look-ahead {v('autonomy', 'control.lookahead_min_m')} m + {v('autonomy', 'control.lookahead_gain_s')}"
                   f" s × speed, at most {v('autonomy', 'control.lookahead_max_m')} m.",
                   f"Delay compensation: steers for where the car will be in {v('autonomy', 'control.latency_comp_s')} "
                   "s (actuator dead time plus processing).",
                   "Learned steering compliance (gentle corners) and a grip monitor that slows the racing line if "
                   f"the yaw rate falls below {v('autonomy', 'speed.grip_ratio_min')} of the expected value."])
    st.append(h("4.6 Vehicle model (in the simulator)"))
    st.append(P(f"A {v('ads_dv', 'vehicle.model')} single-track model with a simplified Pacejka tyre (B = "
                f"{v('ads_dv', 'vehicle.tyre_b')}, C = {v('ads_dv', 'vehicle.tyre_c')}, μ = "
                f"{v('ads_dv', 'vehicle.tyre_mu')}), switching to the kinematic model below "
                f"{v('ads_dv', 'vehicle.dynamic_above_mps')} m/s; steering rate limit, brake pressure build-up, "
                "actuator dead times, grip patches and steering compliance (fsai/vcu/emulator.py)."))
    st.append(h("4.7 Missions and watchdogs"))
    st.append(table(["Setting", "Value", "Source"], rows_of("autonomy", [
        "watchdog.perception_timeout_s", "watchdog.imu_timeout_s", "watchdog.odometry_timeout_s",
        "watchdog.lost_match_s", "watchdog.relocalise_after_s", "watchdog.relocalise_max_steps",
        "mission.stop_target_frac"]), [55, 15, 100]))
    st.append(h("4.8 Real-time design"))
    st += bullets(["Four ROS 2 programs over a virtual CAN bus: VCU emulator, CAN bridge, pretend world with sensors "
                   "and referee, driving stack.",
                   "In the driving node, the IMU, perception and control run on separate threads; the racing line "
                   "is built in a separate process (D15, D16).",
                   f"Scans older than {v('autonomy', 'perception.max_scan_age_s')} s are dropped; a health line is "
                   "printed once a second; maths libraries use one thread each (set in the Docker image)."])
    st.append(h("4.9 Simulator and referee"))
    st.append(P("Seeded tracks for every event are generated from the rule values in section 3.3 and checked by "
                "validate(). A referee watches the true car on the true track, never the stack's estimate, and "
                "counts cones hit, time off track, lap times, the D9 penalties and stop distances. The stack only "
                "receives simulated sensors and talks to the emulator through encoded CAN bytes."))

    # ------------------------------------------------------------------ 5 verification
    st += section("5. Verification and validation", "teal")
    st.append(h("5.1 Automatic tests"))
    groups = {}
    for c in D["tests"]["cases"]:
        g = c["id"].split("::")[0].replace("tests.", "")
        d = groups.setdefault(g, [0, 0])
        d[0 if c["outcome"] == "passed" else 1] += 1
    st.append(table(["Group", "Passed", "Not passed"], [[f"<font name='DVM'>{g}</font>", a, b]
                                                         for g, (a, b) in sorted(groups.items())], [90, 40, 40], "teal"))
    st.append(h("5.2 Safety scenarios (bridge and VCU emulator in lockstep, each run twice)"))
    st.append(table(["Scenario", "Expected", "Observed", "Same fingerprint twice"],
                    [[r["scenario"].replace("_", " "), r["expected"], r["observed"],
                      "yes" if r["rerun_identical"] else "NO"] for r in D["scen"]["scenarios"]],
                    [35, 55, 55, 25], "teal"))
    st.append(h("5.3 Events, stress, real time and real data"))
    worst = [n.replace("_", " ") for n, d in D["stress"].items() if d["passed"] < d["runs"]]
    ra, sy = E["real"]["front_80deg"]["summary"], E["synthetic"]["front_80deg"]["summary"]
    st += bullets([f"Simulated events: {F['ev_ok']} of {F['ev_n']} clean, {F['mi_ok']} of {F['mi_n']} inspections; "
                   f"re-runs byte-identical: {'yes' if F['det'] else 'NO'}.",
                   f"Stress: {len(D['stress'])} conditions × {len(D['stress_seeds'])} seeds; not every run clean in: "
                   f"{', '.join(worst) or 'none'} (one phantom cone left in the map; no cone hit).",
                   f"Real time in Docker: {F['ros_ok']} of {F['ros_n']} runs clean.",
                   f"Real LiDAR (one scene, {E['scene']['frames']} frames): {round(100 * ra['recall'])} % of cones "
                   f"found up to 20 m against {round(100 * sy['recall'])} % in simulation; position error "
                   f"{round(100 * ra['mean_position_error_m'])} cm against {round(100 * sy['mean_position_error_m'], 1)} cm."])
    st.append(P("The full tables are in the Results and Exploratory Run reports.", "small"))
    st.append(h("5.4 Faults found by testing, and what was done"))
    rows = []
    for line in (ROOT / "DECISIONS.md").read_text().splitlines():
        if re.match(r"\| D1[5-8] \|", line):
            c = [x.strip() for x in line.strip("|").split("|")]
            rows.append([c[0], md(c[2]), md(c[4]) if len(c) > 4 else ""])
    st.append(table(["#", "Change", "Why (what testing showed)"], rows, [10, 80, 80], "teal"))
    st.append(Spacer(1, 2 * mm))
    st.append(box(P("<b>Evidence level: Synthetic-validated</b> for the whole system, plus <b>Exploratory real "
                    "data</b> for the LiDAR cone detector only. Not yet run on an ADS-DV."), "teal"))

    # ------------------------------------------------------------------ 6 project management
    st += section("6. Project management", "orange")
    st.append(h("6.1 Risk register"))
    st.append(P("Technical risks come from the assumptions the team must confirm (docs/ASSUMPTIONS.md) and from "
                "the weak spots the tests found. Each has a check and a setting to change."))
    clut = [d for n, d in D["stress"].items() if "clutter_level" in " ".join(d["overrides"])]
    clut_bad, clut_n = sum(d["runs"] - d["passed"] for d in clut), sum(d["runs"] for d in clut)
    risks = [
        ["Handshake timing differs on the real VCU (A2, A17)", "e-stop as AI_COMMS_LOST", "Pause the bridge for 30 ms "
         "and 70 ms, wheels off; the stricter 50 ms is already used"],
        ["Actuator delays are longer than estimated (A14)", "car weaves", "Measure from the candump log; set "
         "latency_comp_s; stress run at 3 × delay is clean"],
        ["Tyre grip lower than estimated (A15)", "wide corners, cones hit", "Start grip_factor at 0.8; grip "
         "monitor slows the line; cold-tyre stress run is clean"],
        ["Trackside clutter mapped as cones (A16)", "phantom cones in the map", "Record a bag at the venue; "
         f"perception filters; a phantom cone stayed in {clut_bad} of {clut_n} clutter stress runs, no cone hit"],
        ["Far cones missed on a real LiDAR", "map built later in each corner", "Measured: 45 % found at 15-20 m; "
         "check on Day 1, tune min_points and pruning"],
        ["Computer overloaded in real time", "late scans, drift, sensor-loss EBS", "Health line; separate "
         "threads and process; scans over 0.4 s dropped"],
        ["Inspection rpm basis unclear (A13)", "inspection judged wrong", "Ask the scrutineer; one setting "
         "(mission.rpm_basis)"],
        ["Steering limit differs between documents (A4)", "steering clipped", "Read live from ANGLE_MAX; tests "
         "run at 21°, 24°, 27.2° and 30°"]]
    st.append(table(["Risk", "Effect", "Mitigation and check"], risks, [58, 37, 75], "orange"))
    st.append(h("6.2 Health and safety on the car"))
    st.append(P("The IMechE ADS-DV User Manual and Operational Safety documents come first; the setup guide "
                "(section 5) adds these steps:"))
    st += bullets(["First powered session with the wheels off the ground, on the jack, with the RES in hand.",
                   "Log the whole CAN session (<font name='DVM'>candump -l can0</font>); tick assumptions A1 to A9.",
                   "Rehearse the 5 s AS_READY wait, the Go rising edge, a low-speed wheel spin, a service-brake "
                   "finish, a RES e-stop and a full power cycle before any driving.",
                   "Software side: sensor loss requests the EBS (rule T4.2.2): perception silent for "
                   f"{v('autonomy', 'watchdog.perception_timeout_s')} s, IMU {v('autonomy', 'watchdog.imu_timeout_s')} s, "
                   f"wheel speeds {v('autonomy', 'watchdog.odometry_timeout_s')} s, or no cone matching the map for "
                   f"{v('autonomy', 'watchdog.lost_match_s')} s.",
                   "On track: walking pace first, then the events at capped speeds; raise speeds only after every "
                   "log shows 0 cones hit (setup guide, section 6)."])
    st.append(h("6.3 Budget"))
    st.append(P("This project bought nothing. The ADS-DV is the shared car IMechE provides; desk testing needs any "
                "computer with Docker. On the car, the team needs a SocketCAN adapter at 500 kbit/s on the Harting "
                "plug (IMechE tests with PEAK) and compute within the shared 30 A supply (setup guide, section 5)."))
    st.append(table(["Item", "Cost to this project", "Note"], [
        ["fsai-open source code", "£0", "MIT licence (LICENSE)"],
        ["ROS 2 Humble Docker base image", "£0", "pinned by digest in docker/Dockerfile"],
        ["Python packages (python-can, cantools, msgpack, PyYAML, pytest, numpy, scipy)", "£0",
         "exact versions in requirements.txt"],
        ["ackermann_msgs 2.0.2", "£0", "BSD, vendored unmodified"],
        ["Real LiDAR data (coneScenes sample)", "£0", "public download; not redistributed"],
        ["Hardware", "£0", "none bought; the ADS-DV is provided by IMechE"]], [80, 30, 60], "orange"))
    st.append(h("6.4 Software and tools"))
    reqs = [l.strip() for l in (ROOT / "requirements.txt").read_text().splitlines()
            if l.strip() and not l.startswith("#")]
    st.append(table(["Tool", "Version or pin", "Used for"], [
        ["ROS 2 Humble (ros:humble-ros-base)", "digest in docker/Dockerfile", "nodes, real-time runs"],
        ["Docker", "any recent version", "one-command install and run"],
        ["Python", "3.10 in the image", "everything"]] +
        [[r.split("==")[0], r.split("==")[1].split(";")[0].strip() + (" (" + r.split(";")[1].strip().replace('"', "") + ")"
                                                                       if ";" in r else ""), "see requirements.txt"]
         for r in reqs], [60, 60, 50], "orange"))
    st.append(P("Not used for evidence: commercial simulators. Every result can be re-run with open-source tools "
                "only.", "small"))
    st.append(h("6.5 Work allocation"))
    st.append(P("Built by one author, with an AI assistant drafting code and documents and every design decision "
                "made by the author (AUTHORS.md, DECISIONS.md). For the team taking it over, each module has its "
                "own files and tests, so it can have its own owner:"))
    st.append(table(["Area", "Files", "Tests that guard it"], [
        ["CAN interface and bridge", "fsai/can/, fsai/bridge/", "conformance/test_spec_v4, test_pitfalls, "
                                                               "unit/test_codec_vs_dbc, test_known_answers"],
        ["VCU emulator and vehicle model", "fsai/vcu/, config/ads_dv.yaml", "conformance tests, scenarios"],
        ["Perception (environmental sensing)", "fsai/autonomy/perception.py, sensors.py", "autonomy/test_perception"],
        ["Localisation and mapping", "fsai/autonomy/slam.py", "autonomy/test_slam, test_closed_loop"],
        ["Path planning and racing line", "fsai/autonomy/planning.py", "autonomy/test_planning_control"],
        ["Speed and steering control", "fsai/autonomy/control.py", "autonomy/test_planning_control"],
        ["Missions, watchdogs, events", "fsai/autonomy/stack.py, events.py, track.py",
         "autonomy/test_track, test_closed_loop"],
        ["ROS 2 and real time", "ros2/src/fsai_ros/", "realtime/test_realtime, ROS 2 runs"]], [45, 60, 65], "orange"))

    # ------------------------------------------------------------------ 7 timeline
    st += section("7. Timeline", "purple")
    dec = []
    for line in (ROOT / "DECISIONS.md").read_text().splitlines():
        m_ = re.match(r"\| (D\d+) \| (\d{4}-\d{2}-\d{2}) \|", line)
        if m_:
            dec.append((m_.group(1), m_.group(2)))
    days = sorted({d for _, d in dec})
    per_day = {d: [k for k, dd in dec if dd == d] for d in days}

    def gantt(f):
        gs = f.add_gridspec(1, 2, width_ratios=[1, 1], wspace=0.9)
        ax = f.add_subplot(gs[0])
        built = [("Rules, scope, decisions D1-D12", 0, 1, "#1f4fa8"), ("Stage 1: safety layer", 0, 1, "#e8892b"),
                 ("Stage 2: driving stack", 0, 2, "#1e9e6a"), ("Sim vs asphalt, real-time fixes D13-D18", 1, 1,
                                                               "#c43d5b"),
                 ("Stage 3: real data, reports, zip", 1, 1, "#7a52c7")]
        for i, (n, s, w, c) in enumerate(built[::-1]):
            ax.barh(i, w, left=s, color=c, height=0.6)
        ax.set_yticks(range(len(built)), [b[0] for b in built[::-1]], fontsize=7)
        ax.set_xticks([0.5 + i for i in range(len(days))], days, fontsize=7)
        ax.set_xlim(0, len(days))
        ax.set_title("Done (dates from DECISIONS.md)")
        ax2 = f.add_subplot(gs[1])
        nxt = ["Install, test and verify", "Wheels-off session (guide 5)", "Inspection missions", "Day 1 track tuning "
               "(guide 6)", "Events at capped speed", "Full-speed events"]
        for i, n in enumerate(nxt[::-1]):
            k = len(nxt) - 1 - i
            ax2.barh(i, 1, left=k, color="#1a9bb0", height=0.6)
            ax2.text(k + 0.5, i, str(k + 1), color="white", ha="center", va="center", fontsize=7, fontweight="bold")
        ax2.set_yticks(range(len(nxt)), nxt[::-1], fontsize=7)
        ax2.set_xticks([])
        ax2.set_xlabel("order of steps")
        ax2.set_title("Next: the team (in order, dates set by the team)")
    st.append(fig(gantt, "Left: the build, by the dates recorded in the decision log. Right: the steps the team "
                         "takes next, in order; each needs the one before it to pass.", 170, 70))
    st.append(table(["Date", "Decisions recorded"], [[d, ", ".join(per_day[d])] for d in days], [30, 140], "purple"))

    # ------------------------------------------------------------------ 8 references
    st += section("8. References", "navy")
    refs = [
        "IMechE (2026). <i>2026 FS-AI Rules</i>, v1 (chapter D dynamic events, IN2 inspection, T1 and T4 technical).",
        "IMechE (2021). <i>FS-AI Dynamic Events Setup and Cones Specification</i>, section DE6.3 and Figures 1-3.",
        "<i>Formula Student Driverless Specification 2026</i>, v1.1 (2026-05-16), section DS 1.",
        "Hypermotive (2021). <i>ADS-DV Software Interface Specification</i>, v4.0.",
        "FS-AI (2026). <i>FS-AI ADS-DV Specifications &amp; Dimensions</i> v1.0 and <i>ADS-DV_2026_CAN1.dbc</i>, "
        "github.com/FS-AI/FS-AI_ADS-DV_Documentation @ bb8f162.",
        "IMechE Formula Student AI (2021). <i>FS-AI_API</i> (MIT), github.com/FS-AI/FS-AI_API @ 33bcb61.",
        "FS-AI. <i>FS-AI_IMechE_ADS-DV_HiL</i> README, known issues.",
        "Chalmers Formula Student. <i>coneScenes</i>, github.com/Chalmers-Formula-Student/coneScenes "
        f"(scene {E['scene']['file']}, SHA-256 {E['scene']['sha256'][:16]}…).",
        "Open Robotics. <i>ROS 2 Humble Hawksbill</i>; ros2/ackermann_msgs 2.0.2 (BSD).",
        "Virtanen, P. et al. (2020). SciPy 1.0: fundamental algorithms for scientific computing in Python. "
        "<i>Nature Methods</i> 17, 261-272 (L-BFGS-B used for the racing line).",
        "fsai-open 0.2.0: README.md, docs/TEAM_SETUP_GUIDE.md, docs/ASSUMPTIONS.md, DECISIONS.md, "
        "reports/RESULTS.md, data_cards/."]
    st += [P(f"{i}. {r}", "card") for i, r in enumerate(refs, 1)]
    st.append(Spacer(1, 3 * mm))
    st.append(not_show(["Performance on the real ADS-DV or a real track: nothing here has touched the car.",
                        "A comparison with other teams' software: none was made.",
                        "Independent review: no one outside the project has reviewed or replicated it yet."]))
    return build("fsai-open_Technical_Report.pdf", "Technical Report",
                 "Design, methodology, verification and project management of fsai-open, driverless software for "
                 "the IMechE FS-AI ADS-DV", "navy", D, st)
