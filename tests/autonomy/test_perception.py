# fsai-open: tests - LiDAR cone detection, ground fitting and camera colour fusion
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import math

import numpy as np

from fsai.autonomy import config as C
from fsai.autonomy import perception as P
from fsai.autonomy import sensors as S
from fsai.autonomy import track as T

CFG = C.load()
GEO = T.load_geometry()


def _scene():
    xs = np.arange(4.0, 30.0, 5.0)
    cones = [[x, 1.5, T.BLUE] for x in xs] + [[x, -1.5, T.YELLOW] for x in xs] + [[3.0, 1.8, T.BIG_ORANGE]]
    return np.array(cones, float)


def test_lidar_finds_cones_to_25_m_within_10_cm():
    rng = np.random.default_rng(1)
    cones = _scene()
    pts = S.Lidar(CFG, GEO, rng).scan(cones, (0.0, 0.0, 0.0))
    found = P.lidar_cones(pts, CFG, rng)
    xy = np.array([[c.x, c.y] for c in found])
    for cx, cy, _ in cones:
        if math.hypot(cx, cy) <= 25:
            d = np.min(np.hypot(*(xy - [cx, cy]).T))
            assert d < 0.10, (cx, cy, d)
    assert len(found) <= len(cones)                      # no phantom LiDAR cones from the ground


def test_lidar_height_tells_big_orange_from_small():
    rng = np.random.default_rng(2)
    cones = _scene()
    found = P.lidar_cones(S.Lidar(CFG, GEO, rng).scan(cones, (0, 0, 0)), CFG, rng)
    big = [c for c in found if c.colour == T.BIG_ORANGE]
    assert len(big) == 1 and abs(big[0].x - 3.0) < 0.15


def test_ground_fit_recovers_a_tilted_plane():
    rng = np.random.default_rng(3)
    xy = rng.uniform(-20, 20, (4000, 2))
    z = 0.02 * xy[:, 0] - 0.01 * xy[:, 1] - 0.3 + rng.normal(0, 0.01, 4000)
    a, b, c = P.fit_ground(np.column_stack([xy, z]), rng)
    assert abs(a - 0.02) < 0.002 and abs(b + 0.01) < 0.002 and abs(c + 0.3) < 0.01


def test_no_points_no_cones_and_ground_only_no_cones():
    rng = np.random.default_rng(4)
    assert P.lidar_cones(np.zeros((0, 3)), CFG, rng) == []
    pts = S.Lidar(CFG, GEO, rng).scan(np.zeros((0, 3)), (0, 0, 0))
    assert len(pts) > 1000 and P.lidar_cones(pts, CFG, rng) == []


def test_fusion_takes_colour_from_camera_and_keeps_lidar_position():
    lid = [P.Cone(10.0, 1.5, -1, 0.0, "lidar"), P.Cone(10.0, -1.5, -1, 0.0, "lidar")]
    cam = [S.CameraDetection(10.3, 1.6, T.BLUE, 0.9), S.CameraDetection(9.8, -1.4, T.YELLOW, 0.8),
           S.CameraDetection(15.0, 5.0, T.BLUE, 0.4)]      # a phantom with nothing under it
    out = P.fuse(lid, cam, CFG, lidar_range=40.0)
    assert [(c.x, c.y, c.colour) for c in out] == [(10.0, 1.5, T.BLUE), (10.0, -1.5, T.YELLOW)]


def test_camera_errors_grow_with_distance():
    """The pretend camera must be worse far away than near, or the tests above prove little."""
    rng = np.random.default_rng(5)
    cam = S.Camera(CFG, rng)
    near = np.array([[6.0, y, T.BLUE] for y in np.linspace(-3, 3, 7)])
    far = np.array([[20.0, y, T.BLUE] for y in np.linspace(-6, 6, 7)])
    def stats(cones):
        n = wrong = 0
        for _ in range(300):
            for d in cam.detect(cones, (0, 0, 0)):
                n += 1
                wrong += d.colour != T.BLUE
        return n, wrong
    n_near, w_near = stats(near)
    n_far, w_far = stats(far)
    assert n_far < n_near and w_far / max(n_far, 1) > w_near / max(n_near, 1)


# ---------------------------------------------------------------- sim vs asphalt: clutter is not a cone
def test_people_barriers_and_grass_near_the_car_are_not_cones():
    """Upright objects beside the track (FSAI26 venue clutter): a person (1.75 m), a crouching
    photographer (0.9 m, wide), a barrier run (0.8 m) and a grass patch must all be rejected, while a
    real cone next to them is still found."""
    cfg = C.load(sensors__lidar_dropout=0.0)
    geo = T.load_geometry()
    rng = np.random.default_rng(3)
    lid = S.Lidar(cfg, geo, rng)
    cones = np.array([[8.0, 1.5, T.YELLOW]])
    clutter = [[8.0, 5.0, 0.22, 1.75], [10.0, -4.0, 0.32, 0.9]]
    clutter += [[6.0 + 0.25 * k, -6.0, 0.18, 0.8] for k in range(24)]                  # barrier, 6 m long
    clutter += [[5.0 + dx, 4.0 + dy, 0.05, 0.18] for dx in (0, 0.2, 0.4) for dy in (0, 0.2, 0.35)]   # grass
    pts = lid.scan(cones, (0.0, 0.0, 0.0), np.array(clutter))
    found = P.lidar_cones(pts, cfg, rng)
    assert len(found) == 1, [(c.x, c.y, c.height) for c in found]
    assert math.hypot(found[0].x - 8.0, found[0].y - 1.5) < 0.15


def test_lidar_only_blobs_need_the_camera_while_it_works():
    from fsai.autonomy.sensors import CameraDetection
    cfg = C.load()
    blob = P.Cone(10.0, 3.0, P.UNKNOWN, 0.0, "lidar", 0.2, 8)
    cone = P.Cone(9.0, -1.5, P.UNKNOWN, 0.0, "lidar", 0.3, 20)
    cam = [CameraDetection(9.05, -1.5, T.YELLOW, 0.9)]
    out = P.fuse([blob, cone], cam, cfg, 40.0, camera_range=22.0)
    assert [(c.colour, c.source) for c in out] == [(T.YELLOW, "lidar+camera")]
    out = P.fuse([blob, cone], [], cfg, 40.0, camera_range=None)       # camera down: LiDAR-only mode
    assert len(out) == 2
