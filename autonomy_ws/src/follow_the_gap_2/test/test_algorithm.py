"""Exercise the supplied algorithm without starting a ROS graph."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from follow_the_gap_2.follow_the_gap_2_node import DEFAULT_PARAMS, ReactiveFollowGap


def make_node():
    node = ReactiveFollowGap.__new__(ReactiveFollowGap)
    for name, value in DEFAULT_PARAMS.items():
        setattr(node, name, value)
    node._logger = Mock()
    return node


def scan(ranges):
    return SimpleNamespace(ranges=ranges, angle_min=np.radians(-135),
                           angle_increment=np.radians(0.25))


def test_empty_and_blocked_scans_use_slow_forward_fallback():
    node = make_node()
    assert node.compute_drive(scan([])) is None
    assert node.compute_drive(scan(np.zeros(1081))) is None
    steering, speed = node.compute_drive(scan(np.full(1081, 0.01)))
    assert steering == node.prev_steering_angle
    assert speed == 0.5


def test_scan_callback_stops_when_no_gap_is_available():
    node = make_node()
    node.compute_drive = Mock(return_value=None)
    node.publish_drive = Mock()
    node.publish_stop = Mock()
    node._log_status = Mock()
    node.prev_steering_angle = 0.3

    node.scan_callback(scan(np.ones(1081)))

    node.publish_drive.assert_not_called()
    node.publish_stop.assert_called_once_with()
    node._log_status.assert_not_called()


def test_scan_callback_stops_on_invalid_lidar_input():
    node = make_node()
    node._warn_rate_limited = Mock()
    node.compute_drive = Mock()
    node.publish_drive = Mock()
    node.publish_stop = Mock()

    node.scan_callback(scan([]))

    node.compute_drive.assert_not_called()
    node.publish_drive.assert_not_called()
    node.publish_stop.assert_called_once_with()


def test_scan_callback_still_publishes_drive_commands():
    node = make_node()
    node.compute_drive = Mock(return_value=(0.2, 1.5))
    node.publish_drive = Mock()
    node._log_status = Mock()

    node.scan_callback(scan(np.ones(1081)))

    node.publish_drive.assert_called_once_with(0.2, 1.5)
    node._log_status.assert_called_once_with(1.5, 0.2)


def test_open_scan_produces_finite_bounded_command():
    node = make_node()
    for _ in range(10):
        steering, speed = node.compute_drive(scan(np.full(1081, 3.0)))
        assert np.isfinite(steering) and np.isfinite(speed)
        assert abs(steering) <= node.max_steer
        assert 0 <= speed <= node.speed_max


def test_narrow_gap_speed_cannot_exceed_configured_maximum():
    node = make_node()
    # Isolate speed selection for an 80-beam gap, which requests 1.5 m/s.
    node.find_max_gap = Mock(return_value=(280, 359))
    data = scan(np.full(1081, 3.0))
    node.speed_max = 2.0
    assert node.compute_drive(data)[1] == 1.5
    for limit in (1.0, 0.8, 0.0):
        node.speed_max = limit
        assert node.compute_drive(data)[1] == limit


def test_blocked_scan_publishes_zero_speed_and_steering():
    node = make_node()
    node.publish_drive = Mock()
    node.prev_steering_angle = 0.24
    node.scan_callback(scan(np.zeros(1081)))
    node.publish_drive.assert_called_once_with(0.0, 0.0)


def test_single_beam_gap_keeps_target_inside_gap():
    node = make_node()
    ranges = np.array([0., 0., 2., 0., 0.])
    assert node.find_max_gap(ranges) == (2, 2)
    assert node.find_best_point(2, 2, ranges) == 2


def test_invalid_increment_stops():
    data = scan(np.ones(1081))
    data.angle_increment = 0.0
    assert make_node().compute_drive(data) is None


def test_metre_bubble_grows_for_closer_obstacle():
    node = make_node()
    near = node.get_bubble_indices(500, 0.5, 1081, np.radians(0.25))
    far = node.get_bubble_indices(500, 2.0, 1081, np.radians(0.25))
    assert len(near) > len(far)


def test_lidar_cap_stays_at_five_metres_across_corridor_widths():
    node = make_node()
    for distance in (1.0, 6.0, 1.0):
        node.compute_drive(scan(np.full(1081, distance)))
        assert node.max_lidar_range == 5.0
    processed = node.preprocess_lidar(np.full(1081, 8.0))
    assert np.allclose(processed[1:-1], 5.0)
