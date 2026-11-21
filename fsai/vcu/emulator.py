# fsai-open: software copy of the ADS-DV VCU safety state machine (spec v4.0 sections 2.4, 3, 4)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Plain words: a pretend car "brain box" (the VCU) that behaves like the real one's safety rules,
so a team can test start, stop and emergency stops on a desk.

Every rule below cites the spec clause it comes from. Behaviour the spec leaves open is marked
ASSUMPTION and listed in docs/ASSUMPTIONS.md for the team to confirm on the first powered session.

The emulator is deterministic: time only moves when tick() is called (10 ms per tick), so a run
with the same inputs always produces byte-identical frames.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..can import codec
from ..can import signals as S
from ..params import Params

AS_OFF, AS_READY, AS_DRIVING, AS_EBRAKE, AS_FINISHED = 1, 2, 3, 4, 5
DT = 0.01

# SHUTDOWN_CAUSE codes, spec v4.0 section 2.2
CAUSE_AI_REQUEST, CAUSE_EBS, CAUSE_COMMS = 1, 4, 6
CAUSE_AUTO_BRAKING, CAUSE_MISSION_STATUS, CAUSE_BRAKE_PLAUS = 7, 8, 11
CAUSE_SDC_OPEN = 0          # ASSUMPTION: an SDC/RES e-stop is not in the enumerated list, so it reports 0


@dataclass
class CarInputs:
    """The physical switches a person sets on the real car."""
    lvms: bool = True            # low-voltage master switch (power)
    tsms: bool = False           # tractive system master switch
    asms: bool = False           # autonomous system master switch
    ami: int = 0                 # mission picked on the touchscreen (0 = none, 1..7)
    ebs_armed: bool = True
    sdc_closed: bool = True      # False = an e-stop button, RES e-stop or crash sensor broke the circuit
    res_go: bool = False         # RES "Go" switch


@dataclass
class Faults:
    ai_comms_lost: bool = False
    autonomous_braking: bool = False
    mission_status: bool = False
    brake_plausibility: bool = False
    ai_estop_request: bool = False
    ebs: bool = False

    def any(self) -> bool:
        return any(vars(self).values())


@dataclass
class VcuEmulator:
    params: Params = field(default_factory=Params)
    inputs: CarInputs = field(default_factory=CarInputs)
    lifted: bool = False          # raised off the ground (static inspection): wheels turn, the car stays put

    def __post_init__(self):
        self.power_cycle()

    # ------------------------------------------------------------------ lifecycle
    def power_cycle(self):
        """Spec 3.9: after an emergency brake, a power cycle is needed before driving again."""
        self.t = 0.0
        self.state = AS_OFF
        self.faults = Faults()
        self.shutdown_cause = 0
        self.needs_power_cycle = False
        self.hs_vcu = 0
        self.hs_wait = 0
        self.frame_age = {m.frame_id: 10_000 for m in S.AI_TX_WATCHED}
        self.ai = {m.name: codec.decode(m, bytes(m.dlc)) for m in S.AI_TX_WATCHED}
        self.ai_seen = False
        self.ready_since = None
        self.ebrake_since = None
        self.go_prev = self.inputs.res_go
        self.go_signal = 0
        # vehicle state
        self.v = 0.0
        self.x = self.y = self.yaw = 0.0
        self.vy = 0.0                 # lateral velocity at the centre of mass (dynamic model)
        self.r = 0.0                  # yaw rate [rad/s]
        self.steer = 0.0
        self.brake_f = self.brake_r = 0.0
        self.trq_f = self.trq_r = 0.0
        self.pulses = [0.0] * 4
        self._lines: dict = {}        # actuator dead-time lines (steer, drive, brake)
        self.events: list[tuple[float, str]] = []

    def _log(self, text: str):
        self.events.append((round(self.t, 2), text))

    # ------------------------------------------------------------------ AI frames in
    def receive(self, frame_id: int, data: bytes):
        msg = S.BY_ID.get(frame_id)
        if msg is None or msg.sender != "AI" or msg not in S.AI_TX_WATCHED:
            return
        self.ai[msg.name] = codec.decode(msg, data)
        self.frame_age[frame_id] = 0
        self.ai_seen = True

    # ------------------------------------------------------------------ helpers
    @property
    def wheel_rpm(self) -> float:
        return self.v / self.params.vehicle.wheel_radius_m * 60 / (2 * math.pi)

    def _moving(self) -> bool:
        return self.wheel_rpm > self.params.vcu.standstill_rpm

    def _status(self):
        return self.ai["AI2VCU_Status"]

    def _torque_req(self):
        return (self.ai["AI2VCU_Drive_F"]["FRONT_AXLE_TRQ_REQUEST"],
                self.ai["AI2VCU_Drive_R"]["REAR_AXLE_TRQ_REQUEST"])

    def _brake_req(self):
        b = self.ai["AI2VCU_Brake"]
        return b["HYD_PRESS_F_REQ_pct"], b["HYD_PRESS_R_REQ_pct"]

    def _enter(self, state: int, why: str):
        if state == self.state:
            return
        self._log(f"{S.AS_STATE[self.state]} -> {S.AS_STATE[state]}: {why}")
        self.state = state
        if state == AS_READY:
            self.ready_since = self.t
            self.go_prev = self.inputs.res_go     # spec 3.2.3: Go already on must be cycled off/on
            self.go_signal = 0
        if state == AS_EBRAKE:
            self.ebrake_since = self.t
            self.needs_power_cycle = True
            self.go_signal = 0

    def _emergency(self, cause: int, why: str):
        if self.state in (AS_READY, AS_DRIVING, AS_FINISHED):
            if not self.shutdown_cause:
                self.shutdown_cause = cause
            self._enter(AS_EBRAKE, why)

    # ------------------------------------------------------------------ one 10 ms cycle
    def tick(self) -> list[tuple[int, bytes]]:
        self.t = round(self.t + DT, 6)
        p = self.params.vcu
        inp = self.inputs

        if not inp.lvms:                       # power removed
            return []

        # --- spec 2.4 handshake and frame timeouts (only once the tractive system is on)
        fresh_status = self.frame_age[0x510] == 0     # an AI2VCU_Status arrived since the last cycle
        for fid in self.frame_age:
            self.frame_age[fid] += 1
        if inp.tsms:
            if self._status()["HANDSHAKE"] == self.hs_vcu and fresh_status:
                self.hs_vcu ^= 1
                self.hs_wait = 0
            else:
                self.hs_wait += 1
            missed = [hex(f) for f, a in self.frame_age.items() if a > p.comms_timeout_cycles]  # more than comms_timeout_cycles consecutive misses
            lost = self.hs_wait > p.comms_timeout_cycles or bool(missed)
            if lost and not self.faults.ai_comms_lost:
                self._log(f"AI_COMMS_LOST (handshake wait {self.hs_wait} cycles, missed {missed})")
            if lost:
                self.faults.ai_comms_lost = True
            elif self.state == AS_OFF and not self.needs_power_cycle:
                self.faults.ai_comms_lost = False   # ASSUMPTION: not latched while AS_OFF
            if self.faults.ai_comms_lost:
                self._emergency(CAUSE_COMMS, "AI comms lost (spec 2.4)")

        # --- AI e-stop request (spec 2.1 ESTOP_REQUEST, SHUTDOWN_CAUSE 1)
        if self._status()["ESTOP_REQUEST"] == 1:
            if self.state in (AS_READY, AS_DRIVING, AS_FINISHED):
                self.faults.ai_estop_request = True
                self._emergency(CAUSE_AI_REQUEST, "AI requested e-stop")

        # --- shutdown circuit (spec 3.5, 3.7, 3.8)
        if not inp.sdc_closed:
            self._emergency(CAUSE_SDC_OPEN, "shutdown circuit open (e-stop / RES / crash sensor)")
        if self.state != AS_EBRAKE and inp.ebs_armed is False and self.state in (AS_READY, AS_DRIVING):
            self.faults.ebs = True
            self._emergency(CAUSE_EBS, "EBS no longer armed")

        st = self.state
        status = self._status()
        tf, tr = self._torque_req()
        bf, br = self._brake_req()

        if st == AS_OFF:
            ack_ok = (not p.require_selected_ack) or status["MISSION_STATUS"] == 1
            if (inp.tsms and inp.asms and inp.ami != 0 and inp.ebs_armed and inp.sdc_closed
                    and ack_ok and not self.needs_power_cycle and not self.faults.any()):
                self._enter(AS_READY, "TSMS+ASMS on, mission selected, EBS armed (spec 3.1)")

        elif st == AS_READY:
            if not inp.asms:
                self._enter(AS_OFF, "ASMS off (spec 3.6)")
            else:
                cond = (self.t - self.ready_since >= p.ready_min_s - 1e-9 and tf == 0 and tr == 0
                        and self.ai["AI2VCU_Steer"]["STEER_REQUEST"] == 0
                        and status["DIRECTION_REQUEST"] == 0
                        and abs(self.steer) < p.ready_max_actual_steer_deg)
                rising = inp.res_go and not self.go_prev
                if cond and rising:
                    self.go_signal = 1
                    self._enter(AS_DRIVING, "GO rising edge after >=5 s with zero requests (spec 3.2)")

        elif st == AS_DRIVING:
            if not inp.res_go:
                self._emergency(CAUSE_SDC_OPEN, "RES Go switched off (spec 3.7)")
            elif not inp.asms:
                self._emergency(CAUSE_SDC_OPEN, "ASMS off while driving (spec 3.7)")
            elif (tf > 0 or tr > 0) and (bf > 0 or br > 0):
                self.faults.brake_plausibility = True
                self._emergency(CAUSE_BRAKE_PLAUS, "torque and brake requested together (spec 4)")
            elif p.hil_brake_estop_quirk and (bf > 0 or br > 0):
                self._emergency(CAUSE_EBS, "HiL quirk: hydraulic brake request (HiL known issue)")
            elif status["DIRECTION_REQUEST"] == 0 and self._moving():
                self.faults.autonomous_braking = True
                self._emergency(CAUSE_AUTO_BRAKING, "NEUTRAL requested above 10 rpm (spec 4)")
            elif status["MISSION_STATUS"] == 3:
                if self._moving():
                    self.faults.mission_status = True
                    self._emergency(CAUSE_MISSION_STATUS, "FINISHED requested above 10 rpm (spec 4)")
                else:
                    self._enter(AS_FINISHED, "mission FINISHED at standstill (spec 3.3)")

        elif st == AS_FINISHED:
            if not inp.asms:
                self._enter(AS_OFF, "ASMS off (spec 3.4)")

        elif st == AS_EBRAKE:
            if self.t - self.ebrake_since >= p.ebrake_hold_s - 1e-9 and not inp.asms:
                self._enter(AS_OFF, "15 s elapsed and ASMS off (spec 3.9); power cycle needed")

        self.go_prev = inp.res_go
        self._dynamics()
        return self.frames()

    # ------------------------------------------------------------------ simple vehicle physics
    def _delay(self, name: str, value, delay_s: float):
        """Pure dead time: the actuator acts on what was requested delay_s ago."""
        n = int(round(delay_s / DT))
        if n <= 0:
            return value
        line = self._lines.setdefault(name, [value] * n)
        line.append(value)
        return line.pop(0)

    def _dynamics(self):
        veh = self.params.vehicle
        r = veh.wheel_radius_m
        driving = self.state == AS_DRIVING
        # steering actuator (spec 3.2: ignored outside AS_DRIVING; 3.3.4 ignored in FINISHED)
        if driving:
            req = self._delay("steer", self.ai["AI2VCU_Steer"]["STEER_REQUEST"], veh.steer_delay_s)
            target = max(-self.params.vcu.angle_max_deg, min(self.params.vcu.angle_max_deg, req))
            step = veh.steer_rate_deg_s * DT
            self.steer += max(-step, min(step, target - self.steer))
        # longitudinal: speed-limited torque request per driven axle (spec 2.1 AI2VCU_Drive_*)
        force = 0.0
        self.trq_f = self.trq_r = 0.0
        if driving:
            omega = self.v / r
            motor_rpm = omega * 60 / (2 * math.pi) * veh.belt_ratio
            axles = [("front", "AI2VCU_Drive_F", "FRONT"), ("rear", "AI2VCU_Drive_R", "REAR")]
            for axle, msg, pre in axles:
                if axle not in veh.driven_axles:
                    continue
                req, limit_rpm = self._delay(axle, (self.ai[msg][f"{pre}_AXLE_TRQ_REQUEST"],
                                                    self.ai[msg][f"{pre}_MOTOR_SPEED_MAX"]), veh.drive_delay_s)
                cap = min(req, veh.axle_torque_max_nm)
                if omega > 0.1:
                    cap = min(cap, veh.electrical_power_max_w / omega)
                trq = max(-cap, min(cap, 2.0 * (limit_rpm - motor_rpm)))
                if axle == "front":
                    self.trq_f = trq
                else:
                    self.trq_r = trq
                force += trq / r
        # brakes
        if driving:
            bf, br = self._delay("brake", self._brake_req(), veh.brake_delay_s)
        elif self.state == AS_EBRAKE:
            bf = br = 100.0
        else:
            bf = br = 0.0
        self.brake_f += (bf - self.brake_f) * 0.2
        self.brake_r += (br - self.brake_r) * 0.2
        decel = (self.brake_f + self.brake_r) / 200 * veh.brake_decel_at_100pct_mps2
        if self.state == AS_EBRAKE:
            decel = max(decel, veh.ebs_decel_mps2)
        resist = veh.rolling_resistance * 9.81 + 0.5 * 1.2 * veh.drag_area_cda_m2 * self.v ** 2 / veh.mass_kg
        a = force / veh.mass_kg - (decel + resist if self.v > 0 else 0)
        self.v = max(0.0, self.v + a * DT)
        if self.lifted:
            # FSAI26 IN2.2.2: static inspection with the car raised; the wheels turn, the car stays put
            self.r = self.vy = 0.0
        elif veh.model == "dynamic" and self.v > veh.dynamic_above_mps:
            self._single_track(force)
        else:
            # kinematic bicycle for position (rear-axle reference, no tyre slip)
            self.r = self.v / veh.wheelbase_m * math.tan(math.radians(self.steer * veh.steer_gain))
            self.vy = veh.wheelbase_m * veh.cg_from_rear_frac * self.r
            self.yaw += self.r * DT
            self.x += self.v * math.cos(self.yaw) * DT
            self.y += self.v * math.sin(self.yaw) * DT
        rev = self.v / (2 * math.pi * r) * DT
        self.pulses = [(c + rev * veh.wheel_pulses_per_rev) % 65536 for c in self.pulses]

    def _single_track(self, drive_force: float):
        """Dynamic single-track model with saturating tyres (simplified Pacejka), 1 ms sub-steps.
        Lets the car slide wide if the driving software asks for more grip than the tyres have.
        self.v stays the longitudinal speed; x, y stay the rear-axle position."""
        veh = self.params.vehicle
        L = veh.wheelbase_m
        lr = L * veh.cg_from_rear_frac
        lf = L - lr
        m, g = veh.mass_kg, 9.81
        fzf, fzr = m * g * lr / L, m * g * lf / L
        mu, B, C = veh.tyre_mu, veh.tyre_b, veh.tyre_c
        if veh.tyre_mu_variation:                           # grip patches fixed to the ground (bumps, damp, rubber)
            mu *= 1 + veh.tyre_mu_variation * 0.5 * (math.sin(self.x / 7.3 + 1.1) + math.cos(self.y / 5.1))
        delta = math.radians(self.steer * veh.steer_gain)
        fx_r = drive_force                                  # rear-wheel drive force uses rear grip
        fyr_cap = math.sqrt(max(0.0, (mu * fzr) ** 2 - fx_r ** 2))
        h = DT / 10
        for _ in range(10):
            vx = max(self.v, 0.5)
            af = delta - math.atan2(self.vy + lf * self.r, vx)
            ar = -math.atan2(self.vy - lr * self.r, vx)
            fyf = mu * fzf * math.sin(C * math.atan(B * af))
            fyr = fyr_cap * math.sin(C * math.atan(B * ar))
            dvy = (fyf * math.cos(delta) + fyr) / m - vx * self.r
            dr = (lf * fyf * math.cos(delta) - lr * fyr) / veh.yaw_inertia_kgm2
            self.vy += dvy * h
            self.r += dr * h
            vry = self.vy - lr * self.r                     # lateral velocity at the rear axle
            self.x += (vx * math.cos(self.yaw) - vry * math.sin(self.yaw)) * h
            self.y += (vx * math.sin(self.yaw) + vry * math.cos(self.yaw)) * h
            self.yaw += self.r * h

    @property
    def lateral_accel(self) -> float:
        return self.v * self.r

    # ------------------------------------------------------------------ VCU frames out
    def frames(self) -> list[tuple[int, bytes]]:
        inp = self.inputs
        f = self.faults
        rpm = min(1250, round(self.wheel_rpm))
        st = self.state
        status = {
            "HANDSHAKE": self.hs_vcu if inp.tsms else 0,
            "SHUTDOWN_REQUEST": 0,
            "AS_SWITCH_STATUS": int(inp.asms), "TS_SWITCH_STATUS": int(inp.tsms),
            "GO_SIGNAL": self.go_signal if st == AS_DRIVING else 0,
            "STEERING_STATUS": 1 if st in (AS_READY, AS_DRIVING) else 0,
            "AS_STATE": st, "AMI_STATE": inp.ami,
            "FAULT_STATUS": int(f.any()), "WARNING_STATUS": 0,
            "AI_ESTOP_REQUEST": int(f.ai_estop_request), "EBS_FAULT": int(f.ebs),
            "AI_COMMS_LOST": int(f.ai_comms_lost), "AUTONOMOUS_BRAKING_FAULT": int(f.autonomous_braking),
            "MISSION_STATUS_FAULT": int(f.mission_status), "BRAKE_PLAUSIBILITY_FAULT": int(f.brake_plausibility),
            "SHUTDOWN_CAUSE": self.shutdown_cause,
        }
        trq_max = self.params.vehicle.axle_torque_max_nm
        tf_req, tr_req = self._torque_req()
        bf_req, br_req = self._brake_req()
        half = lambda v: round(max(0.0, min(100.0, v)) * 2) / 2
        out = [
            (0x520, codec.encode(S.VCU2AI_STATUS, status)),
            (0x521, codec.encode(S.VCU2AI_DRIVE_F, {"FRONT_AXLE_TRQ": round(self.trq_f, 1),
                                                    "FRONT_AXLE_TRQ_REQUEST": tf_req, "FRONT_AXLE_TRQ_MAX": trq_max})),
            (0x522, codec.encode(S.VCU2AI_DRIVE_R, {"REAR_AXLE_TRQ": round(self.trq_r, 1),
                                                    "REAR_AXLE_TRQ_REQUEST": tr_req, "REAR_AXLE_TRQ_MAX": trq_max})),
            (0x523, codec.encode(S.VCU2AI_STEER, {"ANGLE": round(self.steer, 1),
                                                  "ANGLE_MAX": self.params.vcu.angle_max_deg,
                                                  "ANGLE_REQUEST": self.ai["AI2VCU_Steer"]["STEER_REQUEST"]})),
            (0x524, codec.encode(S.VCU2AI_BRAKE, {"HYD_PRESS_F_pct": half(self.brake_f), "HYD_PRESS_F_REQ_pct": bf_req,
                                                  "HYD_PRESS_R_pct": half(self.brake_r), "HYD_PRESS_R_REQ_pct": br_req,
                                                  "STATUS_BRK": 1,
                                                  "STATUS_EBS": 3 if st == AS_EBRAKE else (2 if inp.ebs_armed else 1)})),
            (0x525, codec.encode(S.VCU2AI_SPEEDS, {k: rpm for k in ("FL_WHEEL_SPEED", "FR_WHEEL_SPEED",
                                                                     "RL_WHEEL_SPEED", "RR_WHEEL_SPEED")})),
            (0x526, codec.encode(S.VCU2AI_WHEEL_COUNTS, dict(zip(
                ("FL_PULSE_COUNT", "FR_PULSE_COUNT", "RL_PULSE_COUNT", "RR_PULSE_COUNT"),
                (int(c) for c in self.pulses))))),
        ]
        return out
