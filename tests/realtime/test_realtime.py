# fsai-open: real-time tests - bridge and emulator on separate threads/processes over a real python-can bus
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
"""Wall-clock tests: real 10 ms loops, real python-can buses, operating-system scheduling jitter.
Timing results depend on the machine, so they are reported in the run manifest as well as asserted
against the 50 ms budget of FS-AI 2026 rules T1.1.8 (not against 10 ms)."""
import threading
import time

import pytest

can = pytest.importorskip("can")

from fsai.bridge.core import BridgeCore, DriveCommand
from fsai.bridge.runner import LoopStats, bridge_step_fn, emulator_step_fn, open_bus, periodic
from fsai.params import load
from fsai.vcu.emulator import AS_DRIVING, AS_FINISHED, VcuEmulator

P = load()
RESULTS = {}


def _mission(bus_a, bus_b, drive_s=6.0):
    vcu = VcuEmulator(P)
    core = BridgeCore(P)
    stop = threading.Event()
    s_vcu, s_ai = LoopStats(), LoopStats()
    threads = [threading.Thread(target=periodic, args=(emulator_step_fn(vcu, bus_a), stop, 0.01, s_vcu)),
               threading.Thread(target=periodic, args=(bridge_step_fn(core, bus_b), stop, 0.01, s_ai))]
    for t in threads:
        t.start()
    try:
        i = vcu.inputs
        i.asms = i.tsms = True
        i.ami = 1
        time.sleep(P.vcu.ready_min_s + 1.0)
        i.res_go = True
        t_end = time.monotonic() + drive_s
        while time.monotonic() < t_end:
            core.set_command(DriveCommand(speed_mps=6), time.monotonic())
            time.sleep(0.01)
        reached_driving = any("AS_DRIVING" in e for _, e in vcu.events)
        core.request_finish()
        t_end = time.monotonic() + 10
        while vcu.state != AS_FINISHED and time.monotonic() < t_end:
            core.set_command(DriveCommand(speed_mps=6), time.monotonic())
            time.sleep(0.01)
    finally:
        stop.set()
        for t in threads:
            t.join()
    return vcu, core, s_vcu.summary(), s_ai.summary(), reached_driving


def test_realtime_full_mission_over_python_can_virtual_bus():
    a = can.Bus(interface="virtual", channel="fsai-rt", receive_own_messages=False)
    b = can.Bus(interface="virtual", channel="fsai-rt", receive_own_messages=False)
    try:
        vcu, core, sv, sa, drove = _mission(a, b)
    finally:
        a.shutdown()
        b.shutdown()
    RESULTS["virtual"] = {"vcu_loop": sv, "bridge_loop": sa}
    assert drove, vcu.events
    assert vcu.state == AS_FINISHED, vcu.events
    assert not vcu.faults.any(), vcu.events
    assert sa["max_period_ms"] < 50, sa           # FSAI26 T1.1.8 budget: 50 ms


def test_realtime_full_mission_over_udp_multicast_bus():
    """The bus used by `./run.sh demo` across Docker containers / processes."""
    try:
        a = open_bus("sim")
        b = open_bus("sim")
    except Exception as exc:                       # no multicast route on this machine
        pytest.skip(f"udp_multicast unavailable: {exc}")
    try:
        vcu, core, sv, sa, drove = _mission(a, b, drive_s=3.0)
    finally:
        a.shutdown()
        b.shutdown()
    RESULTS["udp_multicast"] = {"vcu_loop": sv, "bridge_loop": sa}
    assert drove and vcu.state == AS_FINISHED and not vcu.faults.any(), vcu.events
