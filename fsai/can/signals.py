# fsai-open: the ADS-DV AI<->VCU CAN interface, written as data (one row per signal)
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""The CAN interface between the AI computer and the ADS-DV vehicle control unit (VCU).

Sources (facts only, re-typed; no file is copied):
  * ADS-DV Software Interface Specification v4.0 (Hypermotive, 25 Jun 2021), sections 2.1-2.3.
  * ADS-DV_2026_CAN1.dbc, FS-AI/FS-AI_ADS-DV_Documentation @ bb8f1624936f13c10a3406ea060088f2ffb4b908
    (SHA-256 fdc895edde31f2b8f3c8638e3d341e05e231eb434e36a9311d65a4f243cd4a0a).
  * ADSDV_2021_VCU_AI_interface_v2.dbc, FS-AI/FS-AI_API @ 33bcb61d2519ee1ac4935396d9bf465c6ebc81c1 (MIT).

Every frame is CAN 2.0, 500 kbit/s, Intel (little-endian) byte order, 10 ms cycle unless noted.

Deliberate difference from the 2026 DBC: the DBC declares every 1-bit flag as *signed* (`@1-`),
so a DBC-driven decoder reads a set flag as -1. Spec v4.0 calls these fields Unsigned/Enum with
values 0/1, so this table declares them unsigned. tests/conformance/test_pitfalls.py proves both
facts against the real DBC file.

The table is checked signal-by-signal against both DBC files by tests/unit/test_codec_vs_dbc.py
(independent recomputation with the third-party `cantools` decoder).
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Signal:
    name: str
    start: int            # Intel start bit (LSB position)
    length: int
    signed: bool = False
    scale: float = 1.0
    offset: float = 0.0
    minimum: float | None = None
    maximum: float | None = None
    unit: str = ""
    choices: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Message:
    frame_id: int
    name: str
    dlc: int
    sender: str           # "AI" or "VCU"
    cycle_ms: int
    signals: tuple

    def signal(self, name: str) -> Signal:
        for s in self.signals:
            if s.name == name:
                return s
        raise KeyError(f"{self.name} has no signal {name}")


AS_STATE = {1: "AS_OFF", 2: "AS_READY", 3: "AS_DRIVING", 4: "AS_EMERGENCY_BRAKE", 5: "AS_FINISHED"}
AMI_STATE = {0: "NOT_SELECTED", 1: "ACCELERATION", 2: "SKIDPAD", 3: "AUTOCROSS", 4: "TRACK_DRIVE",
             5: "STATIC_INSPECTION_A", 6: "STATIC_INSPECTION_B", 7: "AUTONOMOUS_DEMO"}
MISSION_STATUS = {0: "NOT_SELECTED", 1: "SELECTED", 2: "RUNNING", 3: "FINISHED"}
DIRECTION = {0: "NEUTRAL", 1: "FORWARD"}
SHUTDOWN_CAUSE = {0: "NO_SHUTDOWN", 1: "AI_COMPUTER_REQUEST", 2: "HVIL_OPEN_FAULT", 3: "HVIL_SHORT_FAULT",
                  4: "EBS_FAULT", 5: "OFFBOARD_CHARGER_FAULT", 6: "AI_COMMS_FAULT", 7: "AUTONOMOUS_BRAKING_FAULT",
                  8: "MISSION_STATUS_FAULT", 9: "CHARGING_PROCEDURE_FAULT", 10: "BMS_FAULT",
                  11: "BRAKE_PLAUSIBILITY_FAULT"}
STATUS_BRK = {0: "INITIALISING", 1: "READY", 2: "SHUTTING_DOWN", 3: "SHUTDOWN_COMPLETE", 4: "FAULT"}
STATUS_EBS = {1: "UNAVAILABLE", 2: "ARMED", 3: "TRIGGERED"}


def _flag(name: str, start: int) -> Signal:
    return Signal(name, start, 1, False, 1, 0, 0, 1)


# ---------------------------------------------------------------- AI -> VCU (the five watched frames)
AI2VCU_STATUS = Message(0x510, "AI2VCU_Status", 8, "AI", 10, (
    _flag("HANDSHAKE", 0),
    _flag("ESTOP_REQUEST", 8),
    Signal("MISSION_STATUS", 12, 2, False, 1, 0, 0, 3, choices=MISSION_STATUS),
    Signal("DIRECTION_REQUEST", 14, 2, False, 1, 0, 0, 3, choices=DIRECTION),
    Signal("LAP_COUNTER", 16, 4, False, 1, 0, 0, 15),
    Signal("CONES_COUNT_ACTUAL", 24, 8, False, 1, 0, 0, 255),
    Signal("CONES_COUNT_ALL", 32, 16, False, 1, 0, 0, 65535),
    Signal("VEH_SPEED_ACTUAL", 48, 8, False, 1, 0, 0, 255, "km/h"),
    Signal("VEH_SPEED_DEMAND", 56, 8, False, 1, 0, 0, 255, "km/h"),
))
AI2VCU_DRIVE_F = Message(0x511, "AI2VCU_Drive_F", 4, "AI", 10, (
    Signal("FRONT_AXLE_TRQ_REQUEST", 0, 16, False, 0.1, 0, 0, 195, "Nm"),
    Signal("FRONT_MOTOR_SPEED_MAX", 16, 16, False, 1, 0, 0, 4000, "rpm"),
))
AI2VCU_DRIVE_R = Message(0x512, "AI2VCU_Drive_R", 4, "AI", 10, (
    Signal("REAR_AXLE_TRQ_REQUEST", 0, 16, False, 0.1, 0, 0, 195, "Nm"),
    Signal("REAR_MOTOR_SPEED_MAX", 16, 16, False, 1, 0, 0, 4000, "rpm"),
))
AI2VCU_STEER = Message(0x513, "AI2VCU_Steer", 2, "AI", 10, (
    Signal("STEER_REQUEST", 0, 16, True, 0.1, 0, -30, 30, "deg"),
))
AI2VCU_BRAKE = Message(0x514, "AI2VCU_Brake", 2, "AI", 10, (
    Signal("HYD_PRESS_F_REQ_pct", 0, 8, False, 0.5, 0, 0, 100, "%"),
    Signal("HYD_PRESS_R_REQ_pct", 8, 8, False, 0.5, 0, 0, 100, "%"),
))
# Optional FSG logging frame sent by the AI computer (spec v4.0 section 2.1).
AI2LOG_DYNAMICS2 = Message(0x501, "AI2LOG_Dynamics2", 6, "AI", 10, (
    Signal("Accel_longitudinal_mps2", 0, 16, True, 0.00195313, 0, -64, 63.998, "m/s^2"),
    Signal("Accel_lateral_mps2", 16, 16, True, 0.00195313, 0, -64, 63.998, "m/s^2"),
    Signal("Yaw_rate_degps", 32, 16, True, 0.0078125, 0, -256, 255.992, "deg/s"),
))

# ---------------------------------------------------------------- VCU -> AI
VCU2AI_STATUS = Message(0x520, "VCU2AI_Status", 8, "VCU", 10, (
    _flag("HANDSHAKE", 0),
    Signal("OP_STATE", 4, 4, False, 1, 0, 0, 15),            # 2026 only, no published enumeration
    _flag("SHUTDOWN_REQUEST", 8),
    _flag("AS_SWITCH_STATUS", 9),
    _flag("TS_SWITCH_STATUS", 10),
    _flag("GO_SIGNAL", 11),
    Signal("STEERING_STATUS", 12, 2, False, 1, 0, 0, 3),
    Signal("AS_STATE", 16, 4, False, 1, 0, 0, 7, choices=AS_STATE),
    Signal("AMI_STATE", 20, 4, False, 1, 0, 0, 15, choices=AMI_STATE),
    _flag("FAULT_STATUS", 24),
    _flag("WARNING_STATUS", 25),
    Signal("TS_STATE", 28, 4, False, 1, 0, 0, 15),           # 2026 only, no published enumeration
    _flag("WARN_BATT_TEMP_HIGH", 32),
    _flag("WARN_BATT_SOC_LOW", 33),
    Signal("EBS_STATE", 36, 4, False, 1, 0, 0, 15),          # 2026 only, no published enumeration
    _flag("AI_ESTOP_REQUEST", 40),
    _flag("HVIL_OPEN_FAULT", 41),
    _flag("HVIL_SHORT_FAULT", 42),
    _flag("EBS_FAULT", 43),
    _flag("OFFBOARD_CHARGER_FAULT", 44),
    _flag("AI_COMMS_LOST", 45),
    _flag("AUTONOMOUS_BRAKING_FAULT", 46),
    _flag("MISSION_STATUS_FAULT", 47),
    _flag("CHARGE_PROCEDURE_FAULT", 48),
    _flag("BMS_FAULT", 49),
    _flag("BRAKE_PLAUSIBILITY_FAULT", 50),
    Signal("SHUTDOWN_CAUSE", 56, 8, False, 1, 0, 0, 255, choices=SHUTDOWN_CAUSE),
))
VCU2AI_DRIVE_F = Message(0x521, "VCU2AI_Drive_F", 6, "VCU", 10, (
    Signal("FRONT_AXLE_TRQ", 0, 16, True, 0.1, 0, -195, 195, "Nm"),
    Signal("FRONT_AXLE_TRQ_REQUEST", 16, 16, False, 0.1, 0, 0, 195, "Nm"),
    Signal("FRONT_AXLE_TRQ_MAX", 32, 16, False, 0.1, 0, 0, 195, "Nm"),
))
VCU2AI_DRIVE_R = Message(0x522, "VCU2AI_Drive_R", 6, "VCU", 10, (
    Signal("REAR_AXLE_TRQ", 0, 16, True, 0.1, 0, -195, 195, "Nm"),
    Signal("REAR_AXLE_TRQ_REQUEST", 16, 16, False, 0.1, 0, 0, 195, "Nm"),
    Signal("REAR_AXLE_TRQ_MAX", 32, 16, False, 0.1, 0, 0, 195, "Nm"),
))
VCU2AI_STEER = Message(0x523, "VCU2AI_Steer", 6, "VCU", 10, (
    Signal("ANGLE", 0, 16, True, 0.1, 0, -30, 30, "deg"),
    Signal("ANGLE_MAX", 16, 16, False, 0.1, 0, 0, 30, "deg"),
    Signal("ANGLE_REQUEST", 32, 16, True, 0.1, 0, -30, 30, "deg"),
))
VCU2AI_BRAKE = Message(0x524, "VCU2AI_Brake", 5, "VCU", 10, (
    Signal("HYD_PRESS_F_pct", 0, 8, False, 0.5, 0, 0, 100, "%"),
    Signal("HYD_PRESS_F_REQ_pct", 8, 8, False, 0.5, 0, 0, 100, "%"),
    Signal("HYD_PRESS_R_pct", 16, 8, False, 0.5, 0, 0, 100, "%"),
    Signal("HYD_PRESS_R_REQ_pct", 24, 8, False, 0.5, 0, 0, 100, "%"),
    Signal("STATUS_BRK", 32, 4, False, 1, 0, 0, 15, choices=STATUS_BRK),
    Signal("STATUS_EBS", 36, 4, False, 1, 0, 0, 15, choices=STATUS_EBS),
))
VCU2AI_SPEEDS = Message(0x525, "VCU2AI_Speeds", 8, "VCU", 10, (
    Signal("FL_WHEEL_SPEED", 0, 16, False, 1, 0, 0, 1250, "rpm"),
    Signal("FR_WHEEL_SPEED", 16, 16, False, 1, 0, 0, 1250, "rpm"),
    Signal("RL_WHEEL_SPEED", 32, 16, False, 1, 0, 0, 1250, "rpm"),
    Signal("RR_WHEEL_SPEED", 48, 16, False, 1, 0, 0, 1250, "rpm"),
))
VCU2AI_WHEEL_COUNTS = Message(0x526, "VCU2AI_Wheel_counts", 8, "VCU", 10, (
    Signal("FL_PULSE_COUNT", 0, 16, False, 1, 0, 0, 65535),
    Signal("FR_PULSE_COUNT", 16, 16, False, 1, 0, 0, 65535),
    Signal("RL_PULSE_COUNT", 32, 16, False, 1, 0, 0, 65535),
    Signal("RR_PULSE_COUNT", 48, 16, False, 1, 0, 0, 65535),
))

AI_TX_WATCHED = (AI2VCU_STATUS, AI2VCU_DRIVE_F, AI2VCU_DRIVE_R, AI2VCU_STEER, AI2VCU_BRAKE)
AI_TX = AI_TX_WATCHED + (AI2LOG_DYNAMICS2,)
VCU_TX = (VCU2AI_STATUS, VCU2AI_DRIVE_F, VCU2AI_DRIVE_R, VCU2AI_STEER, VCU2AI_BRAKE,
          VCU2AI_SPEEDS, VCU2AI_WHEEL_COUNTS)
ALL = AI_TX + VCU_TX
BY_ID = {m.frame_id: m for m in ALL}
BY_NAME = {m.name: m for m in ALL}

# Spec v4.0 section 2.3: the AI computer must never transmit on these IDs (inclusive ranges).
RESERVED_IDS = ((0x000, 0x000), (0x080, 0x084), (0x120, 0x124), (0x181, 0x184), (0x284, 0x284),
                (0x301, 0x301), (0x410, 0x41F), (0x450, 0x470), (0x4E2, 0x4E2), (0x4FD, 0x4FF),
                (0x550, 0x550), (0x581, 0x584), (0x600, 0x640), (0x650, 0x660), (0x700, 0x705))


def is_reserved(frame_id: int) -> bool:
    return any(lo <= frame_id <= hi for lo, hi in RESERVED_IDS)
