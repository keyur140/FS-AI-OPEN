# fsai-open: faithful Python port of the 2021 FS-AI_API command-validation logic, used only to demonstrate its pitfalls
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
#
# Ported from FS-AI/FS-AI_API, FS-AI_API/fs-ai_api.c, fs_ai_api_ai2vcu_set_data() lines 700-800,
# commit 33bcb61d2519ee1ac4935396d9bf465c6ebc81c1. Original: MIT License, Copyright (c) 2021 IMechE
# Formula Student AI (see third_party/dbc/LICENSE-FS-AI_API-MIT.txt).
"""Plain words: a copy of how the official 2021 library checks commands, kept only so the tests can
show, side by side, where it silently zeroes or merges commands and how fsai-open behaves instead.
This module is never used to drive anything.
"""
from dataclasses import dataclass

MOTOR_RATIO = 3.5
MOTOR_MAX_RPM = 4000.0


@dataclass
class Library2021:
    # static variables refreshed only by fs_ai_api_vcu2ai_get_data(); 0 until the first call
    front_trq_max_raw: int = 0
    rear_trq_max_raw: int = 0
    steer_angle_max_raw: int = 0
    wheel_rpm: tuple = (0, 0, 0, 0)

    def get_data(self, front_trq_max_nm, rear_trq_max_nm, angle_max_deg, wheel_rpm=(0, 0, 0, 0)):
        self.front_trq_max_raw = round(front_trq_max_nm * 10)
        self.rear_trq_max_raw = round(rear_trq_max_nm * 10)
        self.steer_angle_max_raw = round(angle_max_deg * 10)
        self.wheel_rpm = tuple(wheel_rpm)

    def set_data(self, steer_deg, axle_speed_rpm, axle_torque_nm, brake_pct):
        """Returns what the 2021 library would put on the wire (front and rear identical by design)."""
        f_max = 0.1 * self.front_trq_max_raw
        r_max = 0.1 * self.rear_trq_max_raw
        s_max = 0.1 * self.steer_angle_max_raw
        fastest = max(self.wheel_rpm)
        for rpm, lim in ((700, 50.0), (600, 85.0), (500, 100.0), (400, 120.0), (300, 150.0)):
            if fastest > rpm:
                f_max, r_max = min(f_max, lim), min(r_max, lim)
                break
        steer = max(-s_max, min(s_max, steer_deg))
        speed = max(0.0, min(MOTOR_MAX_RPM, axle_speed_rpm * MOTOR_RATIO))
        tf = max(0.0, min(f_max, axle_torque_nm))
        tr = max(0.0, min(r_max, axle_torque_nm))
        bf = br = max(0.0, min(100.0, brake_pct))
        if bf > 0 or br > 0:
            tf = tr = 0.0
        return {"steer": steer, "front_speed": speed, "rear_speed": speed, "front_trq": tf, "rear_trq": tr,
                "front_brake": bf, "rear_brake": br}
