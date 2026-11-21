# fsai-open: AI-side CAN bridge logic for the ADS-DV, with the safety interlocks the 2021 library lacks
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: the translator between the team's driving software and the car.

The team says "go 5 m/s, steer 10 degrees" or "finish now". Every 10 ms the bridge turns that into
the five frames the car expects, and it refuses to send anything the car would punish with an
emergency stop. Whatever it changes, it says so in `status["notes"]`; nothing is changed silently.

Pure logic, no I/O and no clock of its own: the caller passes `now` (seconds, monotonic). That makes
every behaviour testable on a desk, deterministic, and identical under ROS 2 or plain Python.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..can import codec
from ..can import signals as S
from ..params import Params

AS_OFF, AS_READY, AS_DRIVING, AS_EBRAKE, AS_FINISHED = 1, 2, 3, 4, 5
NOT_SELECTED, SELECTED, RUNNING, FINISHED = 0, 1, 2, 3
NEUTRAL, FORWARD = 0, 1

# FS-AI_API fs-ai_api.c L738-754: torque cap by fastest wheel rpm (keeps electrical power ~constant)
DERATE_TABLE = ((700, 50.0), (600, 85.0), (500, 100.0), (400, 120.0), (300, 150.0))
FAULT_FLAGS = ("AI_ESTOP_REQUEST", "HVIL_OPEN_FAULT", "HVIL_SHORT_FAULT", "EBS_FAULT", "OFFBOARD_CHARGER_FAULT",
               "AI_COMMS_LOST", "AUTONOMOUS_BRAKING_FAULT", "MISSION_STATUS_FAULT", "CHARGE_PROCEDURE_FAULT",
               "BMS_FAULT", "BRAKE_PLAUSIBILITY_FAULT")


@dataclass
class DriveCommand:
    """What the team's software asks for. Use either speed mode or raw mode.

    Speed mode (recommended): speed_mps (target) + steer_deg, optional accel_mps2 / decel_mps2 limits.
    Raw mode: torque_nm (per driven axle), motor_speed_max_rpm, brake_pct, steer_deg.
    """
    steer_deg: float = 0.0
    speed_mps: float | None = None
    accel_mps2: float | None = None
    decel_mps2: float | None = None
    torque_nm: float | None = None
    motor_speed_max_rpm: float | None = None
    brake_pct: float | None = None
    stamp: float = 0.0


class TransmitGuardError(RuntimeError):
    pass


def guard_tx(frame_id: int):
    """Spec v4.0 section 2.3: the AI must never transmit on a reserved ID."""
    if S.is_reserved(frame_id):
        raise TransmitGuardError(f"refusing to transmit on reserved CAN ID {frame_id:#05x}")


class BridgeCore:
    def __init__(self, params: Params | None = None):
        self.p = params or Params()
        self.rx: dict[str, dict] = {}
        self.rx_time: dict[str, float] = {}
        self.cmd: DriveCommand | None = None
        self.finish_requested = False
        self.estop_requested = False
        self._last_tb = (0.0, 0.0)                  # torque and brake sent last cycle (frame-order guard)
        self.standstill_cycles = 0
        self.finished_sent = False
        self.lap_counter = 0
        self.cones_actual = 0
        self.cones_all = 0
        self.speed_demand_kmh = 0.0
        self.last_status: dict = {}
        self.tx_count = 0
        self.rejected_commands = 0

    # ------------------------------------------------------------------ inputs
    def on_frame(self, frame_id: int, data: bytes, now: float):
        msg = S.BY_ID.get(frame_id)
        if msg is None or msg.sender != "VCU":
            return
        self.rx[msg.name] = codec.decode(msg, data)
        self.rx_time[msg.name] = now

    def set_command(self, cmd: DriveCommand, now: float) -> bool:
        """Accept a command only if every number in it is finite. Returns False (and keeps the old
        command, which then goes stale and brakes the car) if it contains NaN or infinity."""
        for name in ("steer_deg", "speed_mps", "accel_mps2", "decel_mps2", "torque_nm",
                     "motor_speed_max_rpm", "brake_pct"):
            v = getattr(cmd, name)
            if v is not None and not math.isfinite(v):
                self.rejected_commands += 1
                return False
        cmd.stamp = now
        self.cmd = cmd
        return True

    def request_finish(self):
        """Mission done: brake to a stop, then report FINISHED (spec 3.3)."""
        self.finish_requested = True

    def request_estop(self):
        self.estop_requested = True

    def set_logging(self, lap: int | None = None, cones_actual: int | None = None, cones_all: int | None = None):
        """FSG logging fields the 2021 library hard-codes to zero."""
        if lap is not None:
            self.lap_counter = max(0, min(15, int(lap)))
        if cones_actual is not None:
            self.cones_actual = max(0, min(255, int(cones_actual)))
        if cones_all is not None:
            self.cones_all = max(0, min(65535, int(cones_all)))

    # ------------------------------------------------------------------ derived state
    def _get(self, msg: str, sig: str, default=None):
        return self.rx.get(msg, {}).get(sig, default)

    @property
    def as_state(self) -> int | None:
        return self._get("VCU2AI_Status", "AS_STATE")

    def wheel_rpms(self) -> list[float] | None:
        sp = self.rx.get("VCU2AI_Speeds")
        if sp is None:
            return None
        return [sp["FL_WHEEL_SPEED"], sp["FR_WHEEL_SPEED"], sp["RL_WHEEL_SPEED"], sp["RR_WHEEL_SPEED"]]

    def speed_mps(self) -> float:
        rpms = self.wheel_rpms()
        if not rpms:
            return 0.0
        return sum(rpms) / 4 * 2 * math.pi / 60 * self.p.vehicle.wheel_radius_m

    def _at_standstill(self) -> bool:
        rpms = self.wheel_rpms()
        return rpms is not None and max(rpms) <= self.p.vcu.standstill_rpm

    def _torque_cap(self, notes: list) -> float | None:
        """Live limit from the VCU (0x521/0x522), never assumed. None = unknown yet."""
        caps = []
        veh = self.p.vehicle
        if "front" in veh.driven_axles:
            caps.append(self._get("VCU2AI_Drive_F", "FRONT_AXLE_TRQ_MAX"))
        if "rear" in veh.driven_axles:
            caps.append(self._get("VCU2AI_Drive_R", "REAR_AXLE_TRQ_MAX"))
        if any(c is None for c in caps):
            return None
        cap = min(caps)
        if self.p.bridge.torque_power_derate:
            fastest = max(self.wheel_rpms() or [0])
            for rpm, limit in DERATE_TABLE:
                if fastest > rpm:
                    if limit < cap:
                        notes.append(f"torque derated to {limit} Nm above {rpm} wheel rpm")
                    cap = min(cap, limit)
                    break
        return cap

    # ------------------------------------------------------------------ the 10 ms step
    def step(self, now: float) -> list[tuple[int, bytes]]:
        notes: list[str] = []
        st = self.as_state
        vs = self.rx.get("VCU2AI_Status", {})
        ami = vs.get("AMI_STATE", 0)
        angle_max = self._get("VCU2AI_Steer", "ANGLE_MAX")
        trq_cap = self._torque_cap(notes)
        veh = self.p.vehicle

        steer = torque = brake = 0.0
        speed_max = 0.0
        direction = NEUTRAL
        mission = NOT_SELECTED if not ami else SELECTED
        stale = False

        if st in (AS_DRIVING, AS_FINISHED):
            direction = FORWARD                     # never NEUTRAL once driving (spec 4)
        if st == AS_DRIVING:
            mission = RUNNING
            if self.finish_requested:
                brake = self.p.bridge.finish_brake_pct
                steer = self.cmd.steer_deg if self.cmd else 0.0
                self.standstill_cycles = self.standstill_cycles + 1 if self._at_standstill() else 0
                if self.standstill_cycles >= 3 or self.finished_sent:
                    mission = FINISHED              # only at <=10 rpm on every wheel (spec 3.3, 4)
                    self.finished_sent = True
                else:
                    notes.append("finishing: braking to standstill before reporting FINISHED")
            elif self.cmd is None or now - self.cmd.stamp > self.p.bridge.command_timeout_s:
                stale = True
                brake = self.p.bridge.finish_brake_pct
                notes.append("no fresh command from team software: controlled braking (not an e-stop)")
            else:
                steer, torque, speed_max, brake = self._map_command(self.cmd, trq_cap, notes)
        elif st == AS_FINISHED:
            mission = FINISHED
            brake = self.p.bridge.finish_brake_pct  # ASSUMPTION: harmless, VCU ignores it in FINISHED
        elif st == AS_EBRAKE:
            mission = FINISHED if self.finished_sent else (RUNNING if self.finish_requested else mission)

        # --- live limits, never assumed (pitfall: 2021 library clamps to 0 before the first read)
        if angle_max is None:
            if steer:
                notes.append("steer limit not yet received from VCU (0x523): steer held at 0")
            steer = 0.0
        else:
            if abs(steer) > angle_max:
                notes.append(f"steer {steer:.1f} clamped to live limit +/-{angle_max}")
            steer = max(-angle_max, min(angle_max, steer))
        if trq_cap is None:
            if torque:
                notes.append("torque limit not yet received from VCU (0x521/0x522): torque held at 0")
            torque = 0.0
        else:
            torque = max(0.0, min(trq_cap, torque))
        brake = max(0.0, min(100.0, brake))

        # --- interlock: never torque and brake together (spec 4 BRAKE_PLAUSIBILITY_FAULT)
        if brake > 0 and torque > 0:
            notes.append("torque set to 0 because brake is requested (brake plausibility rule)")
            torque = 0.0
        # The five frames of one cycle reach the VCU one after another (drive before brake). Going
        # straight from brake to torque (or back), the VCU could read the new torque while the old brake
        # frame is still its latest: torque + brake together. So every change of side passes through one
        # 10 ms cycle with neither (found in a real-time trackdrive run as a BRAKE_PLAUSIBILITY e-stop).
        prev_t, prev_b = self._last_tb
        if torque > 0 and prev_b > 0:
            notes.append("one cycle with no torque after braking (frame-order guard, brake plausibility)")
            torque = 0.0
        elif brake > 0 and prev_t > 0:
            notes.append("one cycle with no brake after driving (frame-order guard, brake plausibility)")
            brake = 0.0

        # --- quantise to wire resolution, so what we report is exactly what is sent
        steer = round(steer * 10) / 10
        torque = math.floor(torque * 10 + 1e-9) / 10
        brake = round(brake * 2) / 2
        speed_max = max(0.0, min(veh.motor_speed_max_rpm, round(speed_max)))
        if torque == 0:
            speed_max = 0.0
        self._last_tb = (torque, brake)

        hs = vs.get("HANDSHAKE", 0)
        speed_kmh = min(255, round(self.speed_mps() * 3.6))
        status_vals = {
            "HANDSHAKE": hs, "ESTOP_REQUEST": int(self.estop_requested), "MISSION_STATUS": mission,
            "DIRECTION_REQUEST": direction, "LAP_COUNTER": self.lap_counter,
            "CONES_COUNT_ACTUAL": self.cones_actual, "CONES_COUNT_ALL": self.cones_all,
            "VEH_SPEED_ACTUAL": speed_kmh, "VEH_SPEED_DEMAND": min(255, round(self.speed_demand_kmh)),
        }
        t_f = torque if "front" in veh.driven_axles else 0.0
        t_r = torque if "rear" in veh.driven_axles else 0.0
        s_f = speed_max if "front" in veh.driven_axles else 0.0
        s_r = speed_max if "rear" in veh.driven_axles else 0.0
        frames = [
            (0x510, codec.encode(S.AI2VCU_STATUS, status_vals)),
            (0x511, codec.encode(S.AI2VCU_DRIVE_F, {"FRONT_AXLE_TRQ_REQUEST": t_f, "FRONT_MOTOR_SPEED_MAX": s_f})),
            (0x512, codec.encode(S.AI2VCU_DRIVE_R, {"REAR_AXLE_TRQ_REQUEST": t_r, "REAR_MOTOR_SPEED_MAX": s_r})),
            (0x513, codec.encode(S.AI2VCU_STEER, {"STEER_REQUEST": steer})),
            (0x514, codec.encode(S.AI2VCU_BRAKE, {"HYD_PRESS_F_REQ_pct": brake, "HYD_PRESS_R_REQ_pct": brake})),
        ]
        for fid, _ in frames:
            guard_tx(fid)
        self.tx_count += 1
        self.last_status = {
            "t": now, "as_state": S.AS_STATE.get(st, "UNKNOWN (no VCU frame yet)" if st is None else str(st)),
            "ami_state": S.AMI_STATE.get(ami, str(ami)), "mission_status": S.MISSION_STATUS[mission],
            "direction": S.DIRECTION[direction], "go_signal": vs.get("GO_SIGNAL", 0),
            "faults": [f for f in FAULT_FLAGS if vs.get(f)],
            "shutdown_cause": S.SHUTDOWN_CAUSE.get(vs.get("SHUTDOWN_CAUSE", 0), "UNKNOWN"),
            "steer_cmd_deg": steer, "torque_cmd_nm": torque, "motor_speed_max_rpm": speed_max,
            "brake_cmd_pct": brake, "speed_mps": round(self.speed_mps(), 3),
            "angle_max_deg": angle_max, "torque_cap_nm": trq_cap, "command_stale": stale,
            "finish_requested": self.finish_requested, "estop_requested": self.estop_requested,
            "rejected_commands": self.rejected_commands,
            "notes": notes,
        }
        return frames

    # ------------------------------------------------------------------ speed mode -> wire values
    def _map_command(self, c: DriveCommand, trq_cap, notes) -> tuple[float, float, float, float]:
        veh = self.p.vehicle
        r = veh.wheel_radius_m
        n_axles = max(1, len(veh.driven_axles))
        if c.speed_mps is None:                         # raw mode, passed through the same interlocks
            self.speed_demand_kmh = 0.0
            return (c.steer_deg, c.torque_nm or 0.0, c.motor_speed_max_rpm or 0.0, c.brake_pct or 0.0)
        target = max(0.0, c.speed_mps)
        self.speed_demand_kmh = target * 3.6
        v = self.speed_mps()
        rpm_target = target / r * 60 / (2 * math.pi) * veh.belt_ratio
        if target >= v - 0.3:
            # Spec 2.1: the drive request is a speed-limited torque request. Torque ceiling from accel limit.
            accel = c.accel_mps2 if c.accel_mps2 is not None else 3.0
            torque = max(0.0, accel) * veh.mass_kg * r / n_axles
            cap = trq_cap if trq_cap is not None else 0.0
            if torque > cap and trq_cap is not None:
                notes.append(f"torque {torque:.1f} Nm limited to {cap} Nm")
            return (c.steer_deg, torque, rpm_target, 0.0)
        decel = c.decel_mps2 if c.decel_mps2 is not None else min(6.0, 2.0 * (v - target))
        brake = 100.0 * max(0.0, decel) / veh.brake_decel_at_100pct_mps2
        return (c.steer_deg, 0.0, 0.0, brake)
