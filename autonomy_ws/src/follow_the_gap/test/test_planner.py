"""Unit tests for the ROS-free Follow-the-Gap planner."""

from argparse import Namespace

import numpy as np

from follow_the_gap.planner import FollowTheGapPlanner


# A synthetic 270 deg scan at 1 deg resolution: beam 0 at -135 deg.
ANGLE_MIN = np.radians(-135.0)
ANGLE_INC = np.radians(1.0)
N_BEAMS = 271


def make_params():
    """Return a parameter set sized for the synthetic scan above."""
    return Namespace(
        min_drive_speed=0.6,
        emergency_distance=0.3,
        steering_response=1.0,
        vehicle_length=0.568,
        lidar_offset=0.27,
        disparity_threshold=0.3,
        lookahead_distance=1.0,
        path_horizon=2.0,
        max_acceleration=0.5,
        max_lateral_acceleration=1.0,
        vehicle_width=0.296,
        safety_margin=0.05,
        wheelbase=0.324,
        min_gap_distance=0.6,
        stop_distance=0.15,
        reaction_time=0.3,
        braking_deceleration=2.0,
        fov_degrees=180.0,
        preprocess_conv_size=3,
        max_lidar_dist=10.0,
        safe_threshold=5,
        best_point_conv_size=20,
        max_steer=0.5,
        straights_steering_angle=0.1396,
        fast_steering_angle=0.0698,
        corners_speed=0.8,
        straights_speed=1.0,
        fast_speed=1.2,
    )


def test_centre_gap_steers_straight():
    """A deep corridor dead ahead yields near-zero steering and full speed."""
    # Near wall everywhere, with a deep open corridor straight ahead. Beam 135
    # of the full scan points straight forward (angle_min + 135 deg = 0).
    ranges = np.full(N_BEAMS, 1.0)
    ranges[125:146] = 10.0

    planner = FollowTheGapPlanner(make_params())
    steering, speed = planner.plan(ranges, ANGLE_MIN, ANGLE_INC)

    assert abs(steering) < np.radians(10.0)
    assert 0 < speed <= make_params().fast_speed


def test_open_side_steers_toward_it():
    """A deep corridor on the left makes the planner steer left (positive)."""
    # A physically wide opening centred at +45 deg.
    ranges = np.full(N_BEAMS, 1.0)
    ranges[150:211] = 10.0

    planner = FollowTheGapPlanner(make_params())
    steering, _ = planner.plan(ranges, ANGLE_MIN, ANGLE_INC)

    assert steering > 0.0


def test_empty_scan_returns_none():
    """An empty range array produces no command (caller should stop)."""
    planner = FollowTheGapPlanner(make_params())
    assert planner.plan(np.array([]), ANGLE_MIN, ANGLE_INC) is None


def test_clear_scan_steers_straight_without_false_edge_obstacle():
    """A uniform max-range scan has neither an edge obstacle nor steering bias."""
    angle_increment = np.radians(0.25)
    ranges = np.full(1081, 10.0)

    planner = FollowTheGapPlanner(make_params())
    steering, speed = planner.plan(
        ranges, np.radians(-135.0), angle_increment)

    assert abs(steering) < 1e-9
    assert np.isclose(speed, make_params().fast_speed)


def test_best_point_uses_gap_midpoint_for_uniform_ranges():
    """A flat range profile should not bias the target toward the gap edge."""
    planner = FollowTheGapPlanner(make_params())
    ranges = np.full(181, 10.0)

    assert planner.find_best_point(0, 181, ranges) == 90


def test_close_wall_stops():
    """A wall 30 cm ahead cannot be handled by steering at corner speed."""
    angles = np.linspace(-3 * np.pi / 4, 3 * np.pi / 4, 1081)
    ranges = np.minimum(10.0, 0.3 / np.maximum(np.cos(angles), 1e-9))
    result = FollowTheGapPlanner(make_params()).plan(
        ranges, angles[0], angles[1] - angles[0])
    assert result is None or result[1] == 0.0


def test_unknown_forward_readings_request_stop():
    """Partial sensor failures must not crash or be treated as free space."""
    planner = FollowTheGapPlanner(make_params())
    for invalid in (np.nan, -np.inf, 0.0, -1.0):
        ranges = np.full(N_BEAMS, 10.0)
        ranges[135] = invalid
        assert planner.plan(ranges, ANGLE_MIN, ANGLE_INC) is None


def test_positive_infinity_is_clear():
    """No-return readings still permit planning in otherwise valid scans."""
    ranges = np.full(N_BEAMS, 10.0)
    ranges[135] = np.inf
    result = FollowTheGapPlanner(make_params()).plan(ranges, ANGLE_MIN, ANGLE_INC)
    assert result is not None and result[1] > 0


def test_invalid_scan_geometry_requests_stop():
    """Malformed geometry must not divide by zero or yield a drive command."""
    planner = FollowTheGapPlanner(make_params())
    for increment in (0.0, -0.01, np.nan, np.inf):
        assert planner.plan(np.ones(N_BEAMS), ANGLE_MIN, increment) is None


def test_gap_below_threshold_is_rejected():
    """There is no fallback to a gap that fails the minimum width."""
    planner = FollowTheGapPlanner(make_params())
    ranges = np.zeros(30)
    ranges[10:14] = 10.0
    assert planner.find_max_gap(ranges) == (None, None)


def test_sub_vehicle_width_opening_is_rejected():
    """A 20 cm slot in a wall cannot fit the vehicle and clearance margin."""
    angles = np.linspace(-np.pi / 2, np.pi / 2, 721)
    ranges = np.full(721, 0.5)
    ranges[np.abs(angles) < np.arctan(0.1 / 0.5)] = 10.0
    result = FollowTheGapPlanner(make_params()).plan(
        ranges, angles[0], angles[1] - angles[0])
    assert result is None or result[1] == 0.0


def test_speed_reduces_with_distance_and_obeys_braking_budget():
    """A straight obstacle caps speed by reaction and stopping distance."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    speeds = [planner._clearance_speed(np.array([d]), np.array([0.0]), 0.0)
              for d in (0.3, 1.0, 2.0, 5.0)]
    assert speeds[0] == 0
    assert all(a < b for a, b in zip(speeds, speeds[1:]))
    for distance, speed in zip((0.3, 1.0, 2.0, 5.0), speeds):
        braking = speed * p.reaction_time + speed ** 2 / (2 * p.braking_deceleration)
        assert braking <= max(0.0, distance - p.stop_distance) + 1e-9


def test_turning_path_obstacle_limits_speed():
    """An obstacle on the turn arc matters even if straight ahead is clear."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    steering = 0.3
    curvature = np.tan(steering) / p.wheelbase
    distance_along_path = 1.2
    x = np.sin(distance_along_path * curvature) / curvature
    y = (1 - np.cos(distance_along_path * curvature)) / curvature
    ranges, angles = np.array([np.hypot(x, y)]), np.array([np.arctan2(y, x)])
    assert planner._clearance_speed(ranges, angles, steering) < planner._clearance_speed(
        ranges, angles, 0.0)


def test_node_defaults_match_launch_configuration():
    """Direct execution and launch use identical planner tuning."""
    import ast
    from pathlib import Path
    import yaml

    root = Path(__file__).resolve().parents[1]
    tree = ast.parse((root / 'follow_the_gap/follow_the_gap_node.py').read_text())
    defaults = next(ast.literal_eval(node.value) for node in tree.body
                    if isinstance(node, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == 'DEFAULT_PARAMS'
                            for t in node.targets))
    config = yaml.safe_load((root / 'config/params.yaml').read_text())
    configured = config['follow_the_gap_node']['ros__parameters']
    assert all(configured[key] == value for key, value in defaults.items())


def test_front_lidar_reserve_permits_slow_approach_but_stops_close_wall():
    """The deployed reserve permits a cautious approach without erasing stops."""
    from pathlib import Path
    import yaml

    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/params.yaml').read_text())
    planner = FollowTheGapPlanner(Namespace(**config['follow_the_gap_node']['ros__parameters']))
    angles = np.linspace(-3 * np.pi / 4, 3 * np.pi / 4, 1081)
    for distance in (0.3, 0.9):
        ranges = np.minimum(10.0, distance / np.maximum(np.cos(angles), 1e-9))
        result = planner.plan(ranges, angles[0], angles[1] - angles[0])
        if distance == 0.3:
            assert result is None or result[1] == 0.0
        else:
            assert result is not None and 0.0 < result[1] < 1.5


def test_disparity_extends_only_into_far_side_and_is_symmetric():
    """A depth edge must not leave an apparent opening beside an obstacle."""
    planner = FollowTheGapPlanner(make_params())
    ranges = np.r_[np.ones(90), np.full(91, 5.0)]
    extended = planner.extend_disparities(ranges, ANGLE_INC)
    assert np.all(extended <= ranges)
    assert np.all(extended[:90] == 1.0)
    assert extended[90] == 1.0 and extended[-1] == 5.0
    mirrored = planner.extend_disparities(ranges[::-1], ANGLE_INC)
    np.testing.assert_allclose(mirrored, extended[::-1])


def test_straight_corridor_does_not_saturate_steering():
    """Parallel walls with adequate width should permit straight driving."""
    angles = np.linspace(-3 * np.pi / 4, 3 * np.pi / 4, 1081)
    ranges = np.minimum(10.0, 0.6 / np.maximum(np.abs(np.sin(angles)), 1e-9))
    planner = FollowTheGapPlanner(make_params())
    steering, speed = planner.plan(ranges, angles[0], angles[1] - angles[0])
    assert abs(steering) < np.radians(3)
    assert speed > 0


def test_acceleration_is_limited_but_emergency_stop_is_immediate():
    """A sudden opening cannot create the recorded speed jump."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    ranges = np.full(N_BEAMS, 10.0)
    previous = 0.0
    for _ in range(20):
        _, speed = planner.plan(ranges, ANGLE_MIN, ANGLE_INC, dt=0.025)
        if previous == 0:
            assert speed == p.min_drive_speed
        else:
            assert 0 <= speed - previous <= p.max_acceleration * 0.025 + 1e-9
        previous = speed
    assert previous > 0
    ranges[135] = 0.2
    result = planner.plan(ranges, ANGLE_MIN, ANGLE_INC, dt=0.025)
    assert result is None or result[1] == 0
    assert planner.previous_speed == 0


def test_long_callback_gap_does_not_allow_large_acceleration():
    """Resuming after a delay must still start slowly."""
    p = make_params()
    result = FollowTheGapPlanner(p).plan(
        np.full(N_BEAMS, 10.0), ANGLE_MIN, ANGLE_INC, dt=5.0)
    assert result[1] == p.min_drive_speed


def test_reduced_steering_response_changes_collision_predictions():
    """Half steering response cannot be modelled as a full-angle turn."""
    p = make_params()
    full = FollowTheGapPlanner(p)
    half_params = Namespace(**vars(p))
    half_params.steering_response = 0.5
    half = FollowTheGapPlanner(half_params)
    steering = p.max_steer
    travel = 1.0
    curvature = np.tan(steering) / p.wheelbase
    x = np.sin(travel * curvature) / curvature + p.lidar_offset * (
        np.cos(travel * curvature) - 1)
    y = (1 - np.cos(travel * curvature)) / curvature + p.lidar_offset * np.sin(
        travel * curvature)
    ranges, angles = np.array([np.hypot(x, y)]), np.array([np.arctan2(y, x)])
    assert full._path_clearance(ranges, angles, steering) < half._path_clearance(
        ranges, angles, steering)


def test_lookup_clearance_is_conservative_against_dense_footprint_sweep():
    """Grid quantisation and pose spacing cannot conceal a collision."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    # Asymmetric fixed obstacles beside and in front of the vehicle.
    obstacles = np.array([[0.7, 0.2], [1.1, -0.35], [-0.25, 0.4], [1.5, 0.8]])
    ranges = np.linalg.norm(obstacles, axis=1)
    angles = np.arctan2(obstacles[:, 1], obstacles[:, 0])
    limits = planner._candidate_clearances(ranges, angles)
    travel = np.arange(0, p.path_horizon, 0.002)
    for steering, limit in zip(planner._steerings, limits):
        curvature = np.tan(steering * p.steering_response) / p.wheelbase
        yaw = travel * curvature
        if abs(curvature) < 1e-9:
            px, py = travel, np.zeros_like(travel)
        else:
            px = np.sin(yaw) / curvature + p.lidar_offset * (np.cos(yaw) - 1)
            py = (1 - np.cos(yaw)) / curvature + p.lidar_offset * np.sin(yaw)
        dx, dy = obstacles[:, 0] - px[:, None], obstacles[:, 1] - py[:, None]
        x = dx * np.cos(yaw)[:, None] + dy * np.sin(yaw)[:, None]
        y = -dx * np.sin(yaw)[:, None] + dy * np.cos(yaw)[:, None]
        collision = np.any((x <= p.safety_margin)
                           & (x >= -p.vehicle_length - p.safety_margin)
                           & (abs(y) <= p.vehicle_width / 2 + p.safety_margin), axis=1)
        first_collision = travel[collision][0] if np.any(collision) else p.path_horizon
        assert limit <= first_collision + 1e-6


def test_paths_reject_body_collision_behind_lidar():
    """Rear-side scan returns must not disappear in the forward crop."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    point = np.array([-0.2, p.vehicle_width / 2 + 0.01])
    limits = planner._candidate_clearances(
        np.array([np.linalg.norm(point)]), np.array([np.arctan2(point[1], point[0])]))
    assert np.all(limits == 0)


def test_speed_floor_never_overrides_braking_clearance():
    """Insufficient clearance gives a stop, not a forced breakaway speed."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    angles = np.linspace(-3 * np.pi / 4, 3 * np.pi / 4, 1081)
    ranges = np.minimum(10.0, 0.4 / np.maximum(np.cos(angles), 1e-9))
    result = planner.plan(ranges, angles[0], angles[1] - angles[0], dt=0.025)
    assert result is None or result[1] == 0
    assert planner.last_diagnostics['reason'] in ('no_gap', 'no_path_at_min_speed')


def test_outputs_never_command_stalling_speeds():
    """An available path produces at least the configured usable speed."""
    p = make_params()
    planner = FollowTheGapPlanner(p)
    angles = np.linspace(-3 * np.pi / 4, 3 * np.pi / 4, 1081)
    for distance in np.linspace(0.3, 3.0, 20):
        ranges = np.minimum(10.0, distance / np.maximum(np.cos(angles), 1e-9))
        result = planner.plan(ranges, angles[0], angles[1] - angles[0], dt=0.025)
        assert result is None or result[1] == 0 or result[1] >= p.min_drive_speed
