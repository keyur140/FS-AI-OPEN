# Assumptions to confirm on the real car

**Author:** KEYUR SHAILESHKUMAR PATEL · ORCID https://orcid.org/0009-0008-5261-043X · *Conducted independently; not an official Brunel University London project.* · Drafted with an AI assistant (Claude, Anthropic); decisions by the author.

**In plain words:** the official documents leave a few things unclear or out of date. Wherever the emulator had to choose, the choice is written down here, together with how to check it on the car. Each item is a parameter in `config/ads_dv.yaml` or a single line in `fsai/vcu/emulator.py`, so it is quick to correct.

| # | What we assumed | Why | How to confirm (first powered, wheels-off session) | Where to change |
|---|---|---|---|---|
| A1 | AS_READY must last at least 5 s before Go is accepted | Spec v4.0 §3.2 (2021) | Time from AS_READY to the earliest accepted Go, from a `candump` log | `vcu.ready_min_s` |
| A2 | 5 missed cycles (50 ms) raise AI_COMMS_LOST | FS-AI 2026 rules T1.1.8 (spec v4.0 §2.4 says 100 ms; strictest used) | Pause the bridge with `kill -STOP` for 30 ms and then 70 ms, **wheels off** | `vcu.comms_timeout_cycles` |
| A3 | Emergency-brake hold lasts 15 s | Spec v4.0 §3.9 | Watch the touchscreen timer after a RES e-stop | `vcu.ebrake_hold_s` |
| A4 | Steering limit is whatever ANGLE_MAX (0x523) reports live | The documents say 21°, 24°, 27.2° or 30°. The bridge never assumes a value, and tests run at all four | Log ANGLE_MAX once | `vcu.angle_max_deg` (emulator only) |
| A5 | MISSION_STATUS=SELECTED is not required to enter AS_READY | §3.1 lists 4 conditions; SELECTED is described as "should". The bridge sends it anyway, and a strict variant is tested | Check whether READY is reached with the bridge stopped | `vcu.require_selected_ack` |
| A6 | AI_COMMS_LOST is not latched while in AS_OFF | §2.4 says only that the e-stop happens outside AS_OFF | Start the bridge after switching TSMS on, then check whether READY is still reachable | `emulator.py`, marked ASSUMPTION |
| A7 | SDC, RES or ASMS e-stops report SHUTDOWN_CAUSE 0 | These causes are not in the enumerated list of §2.2 | Read SHUTDOWN_CAUSE after a RES e-stop | `CAUSE_SDC_OPEN` |
| A8 | The car is rear-wheel drive, and the front-axle frames are sent as zero | 2026 spec sheet says "RWD, open diff". The front frames may be vestigial | Log 0x521 FRONT_AXLE_TRQ while driving slowly | `vehicle.driven_axles` |
| A9 | The real car does **not** e-stop on a hydraulic brake request; the CarMaker HiL does | HiL README, "known issues". Both behaviours are modelled and tested | First slow-speed finish with the service brake | `vcu.hil_brake_estop_quirk` |
| A10 | Mass, drag, brake strength, steering rate and pulses per wheel turn are estimates | Not published | Measure them; only the emulator's physics uses them | `vehicle.*` (marked `status: estimate`) |
| A11 | OP_STATE, TS_STATE and EBS_STATE (new in 2026) are decoded but not interpreted | No published value list | Ask the organisers for the value tables | `fsai/can/signals.py` |
| A12 | The 1-bit flags are unsigned (0/1), not the DBC's signed form (−1) | Spec v4.0 says enum 0/1. The DBC declares them signed, which looks like an error | Ask the organisers to fix the DBC; a test already shows both readings | `fsai/can/signals.py` |
| A13 | "Drivetrain rpm" in Static Inspection A/B (FS-AI 2026 IN2.2.2) means the wheel rpm the VCU reports | The rules do not say wheel or motor | Ask the scrutineer; motor rpm = wheel rpm x 3.5 | `mission.rpm_basis` |
| A14 | Actuator dead times are 0.04 s steering, 0.03 s drive, 0.04 s brake | Estimates; not published | Measure from the `candump` log (request frame to measured response), guide section 6 | `vehicle.steer_delay_s`, `drive_delay_s`, `brake_delay_s`, `control.latency_comp_s` |
| A15 | Tyre grip is 1.2 and the road wheels turn as asked | Estimates for warm semi-slicks on dry tarmac | Watch `grip_ratio` in `/fsai/autonomy/status`; start with `grip_factor` 0.8 | `vehicle.tyre_mu`, `vehicle.steer_gain`, `speed.grip_factor` |
| A16 | Trackside clutter looks like the simulator's (barriers, people, grass, shimmer) | Simulated, not measured | Record a bag at the venue and check the map, guide section 6 | `perception.*` filters |
| A17 | The handshake must be answered within 50 ms, not the 100 ms of spec v4.0 | FS-AI 2026 rules T1.1.8; the stricter value is used | Pause the bridge for 30 ms and then 70 ms on stands, A2 | `vcu.comms_timeout_cycles` |
