# fsai-open: tests - seeded track generator follows the cone layout rules and is deterministic
# Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X)
# Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science,
#   Brunel University London, UK; (2) Independent researcher
# Conducted independently; not an official Brunel University London project.
# Drafted with an AI assistant (Claude, Anthropic); design decisions by the author.
import math

import numpy as np
import pytest

from fsai.autonomy import track as T

GEO = T.load_geometry()


@pytest.mark.parametrize("event", list(T.GENERATORS))
@pytest.mark.parametrize("seed", range(12))
def test_every_generated_track_is_valid(event, seed):
    tr = T.GENERATORS[event](seed, GEO)
    assert T.validate(tr, GEO) == []


@pytest.mark.parametrize("event", list(T.GENERATORS))
def test_same_seed_same_track_and_different_seed_differs(event):
    a, b = T.GENERATORS[event](4, GEO), T.GENERATORS[event](4, GEO)
    assert np.array_equal(a.cones, b.cones) and np.array_equal(a.centreline, b.centreline)
    c = T.GENERATORS[event](5, GEO)
    assert not np.array_equal(a.cones, c.cones)


def test_ds26_colours_blue_left_yellow_right_on_circuits():
    tr = T.trackdrive(0, GEO)
    assert len(tr.colour(T.BLUE)) > 20 and len(tr.colour(T.YELLOW)) > 20
    assert len(tr.colour(T.BIG_ORANGE)) == 4           # one timing line, cones before and after, both sides


def test_validator_catches_a_wrong_side_cone_and_a_big_gap():
    """Test the test: break a valid track two ways; the validator must see both."""
    tr = T.trackdrive(0, GEO)
    cones = tr.cones.copy()
    blue = np.nonzero(cones[:, 2] == T.BLUE)[0]
    cones[blue[3], 2] = T.YELLOW                       # a blue cone painted yellow: wrong side
    cones = np.delete(cones, blue[10:13], axis=0)      # three blue cones missing: gap > 5 m
    broken = T.Track(tr.kind, tr.seed, cones, tr.centreline, True, tr.start_line, tr.finish_line, tr.width_m)
    problems = T.validate(broken, GEO)
    assert any("wrong side" in p for p in problems)
    assert any("gap" in p for p in problems)


def test_skidpad_geometry_matches_fsai_2026_rules():
    """FSAI26 D4.1.2-D4.1.3 and D4.3.3: centres 18.25 m apart, inner 15.25 m and outer 21.25 m
    diameters, 16 cones inside each inner circle, 13 outside each outer circle, nose staged 15 m
    before the timekeeping line."""
    tr = T.skidpad(0, GEO)
    g = GEO["skidpad"]
    cr, cl = np.array(tr.meta["centre_right"]), np.array(tr.meta["centre_left"])
    assert math.isclose(np.hypot(*(cr - cl)), 18.25)
    half = GEO["cones"]["small_base_m"] / 2
    for centre, inner_colour, outer_colour in ((cr, T.YELLOW, T.BLUE), (cl, T.BLUE, T.YELLOW)):
        pts = np.vstack([tr.colour(inner_colour), tr.colour(T.BIG_ORANGE)])
        d = np.hypot(*(pts - centre).T)
        inner = d[d < g["inner_diameter_m"] / 2 + 0.5]
        assert len(inner) == 16, len(inner)
        assert np.allclose(inner, g["inner_diameter_m"] / 2 - half, atol=0.2)
        d = np.hypot(*(tr.colour(outer_colour) - centre).T)
        outer = d[np.abs(d - (g["outer_diameter_m"] / 2 + half)) < 0.25]
        assert len(outer) == 13, len(outer)
    assert math.isclose(tr.meta["timing_x"] - GEO["car"]["nose_m"], 15.0)
    assert tr.meta["laps_per_side"] == g["laps_per_side"] == 2


def test_acceleration_length_and_start_clear_of_the_car():
    tr = T.acceleration(0, GEO)
    (x1, _), _ = tr.start_line
    (x2, _), _ = tr.finish_line
    assert math.isclose(x2 - x1, GEO["acceleration"]["length_m"])
    assert x1 > 2.815 - 0.64                          # the staged car's nose is behind the start line


def test_trackdrive_minimum_radius_and_length():
    for s in range(6):
        tr = T.trackdrive(s, GEO)
        k = np.abs(T.curvature(tr.centreline, True)).max()
        assert 1 / k >= GEO["trackdrive"]["min_radius_m"] - 1e-6
        assert GEO["trackdrive"]["length_min_m"] - 1 <= tr.length_m <= GEO["trackdrive"]["length_max_m"] + 1
