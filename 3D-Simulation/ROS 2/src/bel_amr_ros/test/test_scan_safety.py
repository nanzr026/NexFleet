"""Blocked-path recovery cannot authorize forward motion or blind rotation."""

import math

from bel_amr_ros.scan_safety import recovery_rotation, summarize_scan


def test_actual_beam_and_invalid_returns():
    ranges = [math.inf] * 361
    ranges[190] = 0.7
    result = summarize_scan(ranges, -math.pi, math.pi / 180, 12)
    assert math.isclose(result['beam'], math.radians(10))
    assert result['distance'] == 0.7
    assert result['rotation_valid']
    ranges[180] = math.nan
    result = summarize_scan(ranges, -math.pi, math.pi / 180, 12)
    assert not result['front_valid']
    assert not result['rotation_valid']


def test_only_safe_turn_away_is_allowed():
    assert recovery_rotation(1.5, 0.0, 0.7, True) == 0.25
    assert recovery_rotation(1.5, 0.2, 0.7, True) == -0.25
    assert recovery_rotation(0.1, 0.0, 0.7, True) == 0
    assert recovery_rotation(1.5, 0.0, 0.5, True) == 0
    assert recovery_rotation(1.5, 0.0, 0.7, False) == 0


def test_partial_scan_cannot_authorize_swept_footprint_rotation():
    result = summarize_scan([12.] * 181, -math.pi / 2, math.pi / 180, 12)
    assert result['front_valid']
    assert not result['rotation_valid']
