# fsai-open: tests - EKF-SLAM builds a correct map, removes phantom cones and learns the gyro bias
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import math

import numpy as np

from fsai.autonomy import config as C
from fsai.autonomy.perception import Cone
from fsai.autonomy.sensors import world_to_vehicle
from fsai.autonomy.slam import EkfSlam

CFG = C.with_overrides(C.load(), slam__rear_slip_gain=0.0)


def _drive(slam, cones, rng, seconds=12.0, v=5.0, yaw_rate=0.0, bias=0.0, phantom_every=0, fov=None):
    x = y = th = 0.0
    for k in range(int(seconds / 0.01)):
        th += yaw_rate * 0.01
        x += v * math.cos(th) * 0.01
        y += v * math.sin(th) * 0.01
        slam.predict(v * (1 + rng.normal(0, 0.01)), yaw_rate + bias + rng.normal(0, math.radians(0.3)), 0.01)
        if k % 10 == 0:
            rel = world_to_vehicle(cones[:, :2], (x, y, th))
            vis = (rel[:, 0] > 1.0) & (np.hypot(*rel.T) < 20) & (np.abs(np.arctan2(rel[:, 1], rel[:, 0])) < 0.8)
            obs = [Cone(float(a + rng.normal(0, 0.03)), float(b + rng.normal(0, 0.03)), int(c), 0.9, "lidar+camera")
                   for (a, b), c in zip(rel[vis], cones[vis, 2])]
            if phantom_every and k % (10 * phantom_every) == 0:
                obs.append(Cone(float(rng.uniform(3, 15)), float(rng.uniform(-6, 6)), 0, 0.5, "camera"))
            slam.update(obs, fov_check=fov)
    return x, y, th


def _corridor():
    xs = np.arange(3.0, 80.0, 4.0)
    return np.array([[x, 1.5, 0] for x in xs] + [[x, -1.5, 1] for x in xs], float)


def test_map_matches_truth_and_pose_is_accurate():
    rng = np.random.default_rng(0)
    cones = _corridor()
    slam = EkfSlam(CFG)
    x, y, _ = _drive(slam, cones, rng)
    m = slam.cone_map()
    d = np.min(np.hypot(m[:, None, 0] - cones[None, :, 0], m[:, None, 1] - cones[None, :, 1]), axis=1)
    assert d.max() < 0.3 and len(m) >= 25
    assert math.hypot(slam.pose[0] - x, slam.pose[1] - y) < 0.3
    assert set(np.unique(m[:, 2]).astype(int)) <= {0, 1}


def test_phantom_cones_are_pruned():
    rng = np.random.default_rng(1)
    slam = EkfSlam(CFG)
    fov = lambda p: (p[:, 0] > 1.0) & (np.abs(np.arctan2(p[:, 1], p[:, 0])) < 0.8)
    _drive(slam, _corridor(), rng, phantom_every=3, fov=fov)
    m = slam.cone_map()
    cones = _corridor()
    d = np.min(np.hypot(m[:, None, 0] - cones[None, :, 0], m[:, None, 1] - cones[None, :, 1]), axis=1)
    assert slam.stats["pruned"] > 0
    assert np.sum(d > 1.0) <= 1                         # at most one phantom survives in the confirmed map


def test_gyro_bias_is_learned():
    rng = np.random.default_rng(2)
    slam = EkfSlam(CFG)
    _drive(slam, _corridor(), rng, seconds=14, bias=math.radians(0.5))
    assert abs(math.degrees(slam.mu[3]) - 0.5) < 0.2


def test_without_cones_the_pose_drifts_with_the_bias():
    """Test the test: with no landmarks the same bias must cause visible heading drift."""
    rng = np.random.default_rng(3)
    slam = EkfSlam(CFG)
    _, _, th = _drive(slam, np.zeros((0, 3)), rng, seconds=14, bias=math.radians(0.5))
    assert abs(math.degrees(slam.pose[2] - th)) > 5.0


def test_freeze_keeps_only_confirmed_and_stops_growing():
    rng = np.random.default_rng(4)
    slam = EkfSlam(CFG)
    _drive(slam, _corridor(), rng, seconds=6)
    slam.freeze()
    n = slam.n
    assert slam.frozen and np.all(slam.confirmed)
    _drive(slam, _corridor() + [0, 0, 0], rng, seconds=2)
    assert slam.n == n
