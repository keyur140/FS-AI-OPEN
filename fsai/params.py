# fsai-open: load the sourced ADS-DV parameter file into plain dataclasses
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
from __future__ import annotations

from dataclasses import dataclass, fields, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FILE = ROOT / "config" / "ads_dv.yaml"


@dataclass(frozen=True)
class Vehicle:
    wheelbase_m: float = 1.530
    track_m: float = 1.200
    wheel_radius_m: float = 0.2664
    belt_ratio: float = 3.5
    motor_speed_max_rpm: float = 4000
    axle_torque_max_nm: float = 195.0
    electrical_power_max_w: float = 5000
    driven_axles: tuple = ("rear",)
    mass_kg: float = 300.0
    rolling_resistance: float = 0.015
    drag_area_cda_m2: float = 0.9
    brake_decel_at_100pct_mps2: float = 9.0
    ebs_decel_mps2: float = 8.0
    steer_rate_deg_s: float = 60.0
    wheel_pulses_per_rev: int = 20
    overall_width_m: float = 1.430
    overall_length_m: float = 2.815
    rear_overhang_m: float = 0.64
    model: str = "dynamic"
    dynamic_above_mps: float = 3.0
    cg_from_rear_frac: float = 0.5
    yaw_inertia_kgm2: float = 180.0
    tyre_mu: float = 1.2
    tyre_b: float = 10.0
    tyre_c: float = 1.3
    tyre_mu_variation: float = 0.0
    steer_gain: float = 1.0
    steer_delay_s: float = 0.04
    drive_delay_s: float = 0.03
    brake_delay_s: float = 0.04


@dataclass(frozen=True)
class Vcu:
    angle_max_deg: float = 24.0
    ready_min_s: float = 5.0
    comms_timeout_cycles: int = 5
    ebrake_hold_s: float = 15.0
    standstill_rpm: float = 10
    ready_max_actual_steer_deg: float = 5.0
    require_selected_ack: bool = False
    hil_brake_estop_quirk: bool = False


@dataclass(frozen=True)
class Bridge:
    torque_power_derate: bool = True
    finish_brake_pct: float = 50.0
    command_timeout_s: float = 0.2


@dataclass(frozen=True)
class Params:
    vehicle: Vehicle = Vehicle()
    vcu: Vcu = Vcu()
    bridge: Bridge = Bridge()


def _section(cls, raw: dict):
    vals = {}
    for f in fields(cls):
        if f.name in raw:
            v = raw[f.name]["value"] if isinstance(raw[f.name], dict) else raw[f.name]
            vals[f.name] = tuple(v) if isinstance(v, list) else v
    return cls(**vals)


def load(path: Path | str = DEFAULT_FILE, **overrides) -> Params:
    import yaml
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    p = Params(_section(Vehicle, raw.get("vehicle", {})), _section(Vcu, raw.get("vcu", {})),
               _section(Bridge, raw.get("bridge", {})))
    for key, value in overrides.items():          # e.g. vcu__angle_max_deg=30
        sec, name = key.split("__")
        p = replace(p, **{sec: replace(getattr(p, sec), **{name: value})})
    return p
