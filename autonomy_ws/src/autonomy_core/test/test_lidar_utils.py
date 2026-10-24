"""Unit tests for the ROS-free LiDAR helpers."""

import numpy as np

from autonomy_core import lidar_utils


# A synthetic 270 deg scan at 1 deg resolution: beam 0 at -135 deg.
ANGLE_MIN = np.radians(-135.0)
ANGLE_INC = np.radians(1.0)
N_BEAMS = 271


def test_crop_to_fov_keeps_only_front():
    """Cropping a 270 deg scan to 90 deg yields the centred [-45, 45] window."""
    ranges = np.ones(N_BEAMS)
    cropped, angle_of_first = lidar_utils.crop_to_fov(
        ranges, ANGLE_MIN, ANGLE_INC, np.radians(90.0))

    # First kept beam should sit at roughly -45 deg.
    assert np.isclose(np.degrees(angle_of_first), -45.0, atol=1.0)
    # ~91 beams span -45..45 deg at 1 deg resolution.
    assert 89 <= cropped.size <= 93


def test_crop_to_fov_clamps_to_available_beams():
    """Requesting a wider FOV than the sensor has returns the full array."""
    ranges = np.ones(N_BEAMS)
    cropped, _ = lidar_utils.crop_to_fov(
        ranges, ANGLE_MIN, ANGLE_INC, np.radians(360.0))
    assert cropped.size == N_BEAMS


def test_index_to_steering_centre_is_straight():
    """A beam pointing straight ahead produces zero steering."""
    # angle_of_first = -45 deg, increment 1 deg -> index 45 is straight ahead.
    steer = lidar_utils.index_to_steering(
        45, np.radians(-45.0), ANGLE_INC, max_steer=1.0, gain=0.5)
    assert np.isclose(steer, 0.0, atol=1e-6)


def test_index_to_steering_left_is_positive_and_clamped():
    """A left-of-centre beam steers left (positive) and respects the limit."""
    steer = lidar_utils.index_to_steering(
        90, np.radians(-45.0), ANGLE_INC, max_steer=0.2, gain=0.5)
    assert steer > 0.0
    assert steer <= 0.2 + 1e-9


def test_smooth_preserves_constant_ranges_at_scan_edges():
    """Filtering should not turn clear scan boundaries into close obstacles."""
    ranges = np.full(10, 10.0)

    smoothed = lidar_utils.smooth(ranges, 3)

    assert np.array_equal(smoothed, ranges)


def test_smooth_handles_windows_larger_than_scan():
    """A smoothing window wider than the scan still returns one value per beam."""
    ranges = np.array([1.0, 3.0])

    smoothed = lidar_utils.smooth(ranges, 5)

    assert smoothed.shape == ranges.shape
    assert np.all(np.isfinite(smoothed))
