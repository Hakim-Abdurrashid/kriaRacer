"""Follow gaps with disparity extension and feasible Ackermann turn selection."""

from functools import lru_cache

import numpy as np

from autonomy_core import lidar_utils


class FollowTheGapPlanner:
    """ROS-free planner with optional time-based command acceleration limiting."""

    def __init__(self, params):
        """Store geometry and tuning, and initialise command history."""
        positive = ('vehicle_width', 'vehicle_length', 'wheelbase',
                    'min_gap_distance', 'braking_deceleration', 'max_lidar_dist',
                    'lookahead_distance', 'path_horizon', 'max_acceleration',
                    'max_lateral_acceleration', 'disparity_threshold', 'max_steer',
                    'steering_response', 'emergency_distance', 'min_drive_speed')
        nonnegative = ('safety_margin', 'stop_distance', 'reaction_time',
                       'lidar_offset', 'corners_speed', 'straights_speed', 'fast_speed')
        for name in positive + nonnegative:
            value = getattr(params, name)
            if not np.isfinite(value) or value < 0 or (name in positive and value == 0):
                raise ValueError(f'Invalid {name}: {value}')
        if min(params.corners_speed, params.straights_speed, params.fast_speed) < params.min_drive_speed:
            raise ValueError('Every speed ceiling must be at least min_drive_speed')
        if params.lidar_offset > params.vehicle_length:
            raise ValueError('lidar_offset must fit inside vehicle_length')
        self.params = params
        self.fov_rad = np.radians(params.fov_degrees)
        self.previous_speed = 0.0
        self.previous_steering = 0.0
        self.last_diagnostics = {}
        self._steerings = np.linspace(-params.max_steer, params.max_steer, 15)
        self._grid = self._clearance_grid(
            params.vehicle_length, params.vehicle_width, params.lidar_offset,
            params.safety_margin, params.wheelbase, params.steering_response,
            params.max_steer, min(params.path_horizon, params.max_lidar_dist))

    def reset(self):
        """Reset command history after an externally requested stop."""
        self.previous_speed = 0.0
        self.previous_steering = 0.0

    def _stop(self, reason):
        self.reset()
        self.last_diagnostics = {'reason': reason}
        return None

    def plan(self, ranges, angle_min, angle_increment, dt=None):
        """Return steering and speed, or None to stop on unusable input/gaps.

        Supplying elapsed seconds enables command acceleration limiting and
        models the previous steering during the reaction interval. Calls without
        dt are independent, useful for evaluating individual recorded scans.
        """
        p = self.params
        if dt is None:
            self.previous_speed = 0.0
            self.previous_steering = 0.0
        elif not np.isfinite(dt) or dt <= 0:
            return self._stop('invalid_timing')
        ranges = np.asarray(ranges, dtype=float)
        if (ranges.ndim != 1 or not np.isfinite(angle_min)
                or not np.isfinite(angle_increment) or angle_increment <= 0):
            return self._stop('invalid_geometry')
        ranges = np.minimum(ranges, p.max_lidar_dist)
        forward, first = lidar_utils.crop_to_fov(
            ranges, angle_min, angle_increment, self.fov_rad)
        if (forward.size == 0 or not np.all(np.isfinite(forward))
                or np.any(forward <= 0)):
            return self._stop('invalid_scan')

        forward_angles = first + np.arange(len(forward)) * angle_increment
        if self._front_blocked(forward, forward_angles):
            return self._stop('front_emergency')

        # Detect depth edges before smoothing can conceal a thin obstacle.
        extended = self.extend_disparities(forward, angle_increment)
        processed = np.minimum(extended, lidar_utils.smooth(
            extended, p.preprocess_conv_size))
        processed[processed <= p.min_gap_distance] = 0.0
        gaps = self._gaps(processed)
        if not gaps:
            return self._stop('no_gap')

        # Each surviving gap contributes a target, not just the widest gap.
        targets = []
        for start, stop in gaps:
            index = self.find_best_point(start, stop, processed)
            bearing = first + index * angle_increment
            distance = min(processed[index], p.lookahead_distance)
            x = p.lidar_offset + distance * np.cos(bearing)
            y = distance * np.sin(bearing)
            preferred = np.arctan2(2 * p.wheelbase * y, x * x + y * y) / p.steering_response
            targets.append((bearing, float(np.clip(preferred, -p.max_steer, p.max_steer))))

        # Use the complete scan for footprint checks, including rear-side rays.
        all_angles = angle_min + np.arange(len(ranges)) * angle_increment
        valid = np.isfinite(ranges) & (ranges > 0) & (ranges < p.max_lidar_dist)
        obstacle_ranges, obstacle_angles = ranges[valid], all_angles[valid]
        clearances = self._candidate_clearances(obstacle_ranges, obstacle_angles)
        choices = []
        for steering, clearance in zip(self._steerings, clearances):
            # Score traversable distance first. Gap alignment and continuity
            # resolve comparable paths rather than always saturating steering.
            alignment = min(abs(steering - target[1]) for target in targets)
            score = (clearance - 0.25 * alignment
                     - 0.10 * abs(steering - self.previous_steering)
                     - 0.05 * abs(steering))
            speed = min(self._select_speed(steering), self._speed_for_clearance(clearance))
            curvature = abs(np.tan(steering * p.steering_response) / p.wheelbase)
            if curvature > 1e-9:
                speed = min(speed, np.sqrt(p.max_lateral_acceleration / curvature))
            if speed >= p.min_drive_speed:
                choices.append((score, float(steering), clearance, speed))
        if not choices:
            return self._stop('no_path_at_min_speed')
        _, steering, clearance, speed = max(choices, key=lambda item: item[0])
        if dt is not None:
            # Deceleration is never delayed; long callback gaps cannot authorize
            # a large acceleration jump on the next scan.
            # A stopped car needs the measured breakaway command. Once moving,
            # limit subsequent increases; never force this floor through a cap.
            ramp = max(p.min_drive_speed,
                       self.previous_speed + p.max_acceleration * min(dt, 0.1))
            speed = min(speed, ramp)
        self.previous_speed = float(speed)
        self.previous_steering = steering
        self.last_diagnostics = {
            'reason': 'clearance_stop' if speed <= 1e-9 else 'moving',
            'path_clearance': clearance, 'gap_count': len(gaps),
            'target_bearings': [t[0] for t in targets],
        }
        return steering, float(speed)

    def extend_disparities(self, ranges, angle_increment):
        """Extend the near side of each depth jump into its farther neighbour."""
        p = self.params
        source = np.asarray(ranges, dtype=float)
        extended = source.copy()
        half_width = p.vehicle_width / 2 + p.safety_margin
        for index in np.flatnonzero(np.abs(np.diff(source)) > p.disparity_threshold):
            near = index if source[index] < source[index + 1] else index + 1
            distance = source[near]
            count = int(np.ceil(np.arcsin(min(1.0, half_width / distance))
                                / angle_increment))
            if near == index:
                start, stop = near, min(len(source), near + count + 1)
            else:
                start, stop = max(0, near - count), near + 1
            extended[start:stop] = np.minimum(extended[start:stop], distance)
        return extended

    @staticmethod
    def _arc(travel, curvature):
        if abs(curvature) < 1e-9:
            return travel, np.zeros_like(travel), np.zeros_like(travel)
        yaw = travel * curvature
        return np.sin(yaw) / curvature, (1 - np.cos(yaw)) / curvature, yaw

    def _path_clearance(self, ranges, angles, steering_angle):
        """Return collision-free rear-axle travel for a swept rectangular car.

        The LiDAR is at the front of the footprint, lidar_offset ahead of the
        rear-axle/base_link reference. The old steering is held during the
        command-based reaction distance before transitioning to the candidate.
        """
        p = self.params
        horizon = min(p.path_horizon, p.max_lidar_dist)
        step = 0.05
        travel = np.arange(0.0, horizon + step / 2, step)
        delay_distance = self.previous_speed * p.reaction_time
        previous_curvature = np.tan(self.previous_steering * p.steering_response) / p.wheelbase
        curvature = np.tan(steering_angle * p.steering_response) / p.wheelbase
        x1, y1, yaw1 = self._arc(np.minimum(travel, delay_distance), previous_curvature)
        x2, y2, yaw2 = self._arc(np.maximum(0.0, travel - delay_distance), curvature)
        yaw = yaw1 + yaw2
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        path_x = x1 + np.cos(yaw1) * x2 - np.sin(yaw1) * y2
        path_y = y1 + np.sin(yaw1) * x2 + np.cos(yaw1) * y2
        path_x += p.lidar_offset * (cos_yaw - 1)
        path_y += p.lidar_offset * sin_yaw
        # Bound footprint motion between samples, including rotational motion.
        corner_radius = np.hypot(max(p.lidar_offset, p.vehicle_length - p.lidar_offset),
                                 p.vehicle_width / 2)
        padding = step / 2 * (1 + max(abs(curvature), abs(previous_curvature))
                              * corner_radius)
        margin = p.safety_margin + padding
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        # Discard points outside the swept footprint's axis-aligned bounds.
        # This preserves every possible collision while avoiding large matrices.
        centre_x = path_x - p.vehicle_length / 2 * cos_yaw
        centre_y = path_y - p.vehicle_length / 2 * sin_yaw
        extent_x = (p.vehicle_length / 2 + margin) * np.abs(cos_yaw)
        extent_x += (p.vehicle_width / 2 + margin) * np.abs(sin_yaw)
        extent_y = (p.vehicle_length / 2 + margin) * np.abs(sin_yaw)
        extent_y += (p.vehicle_width / 2 + margin) * np.abs(cos_yaw)
        relevant = ((x >= np.min(centre_x - extent_x))
                    & (x <= np.max(centre_x + extent_x))
                    & (y >= np.min(centre_y - extent_y))
                    & (y <= np.max(centre_y + extent_y)))
        dx, dy = x[relevant] - path_x[:, None], y[relevant] - path_y[:, None]
        local_x = dx * cos_yaw[:, None] + dy * sin_yaw[:, None]
        local_y = -dx * sin_yaw[:, None] + dy * cos_yaw[:, None]
        occupied = np.any((local_x <= margin)
                          & (local_x >= -p.vehicle_length - margin)
                          & (np.abs(local_y) <= p.vehicle_width / 2 + margin), axis=1)
        if np.any(occupied):
            return max(0.0, float(travel[occupied][0]) - step)
        return horizon

    @staticmethod
    @lru_cache(maxsize=8)
    def _clearance_grid(length, width, offset, margin, wheelbase, response, max_steer, horizon):
        """Precompute conservative footprint travel limits on a 1 cm grid.

        Each cell stores the first travel distance at which the car could touch
        that cell. Midpoint sampling includes bounds for motion between poses
        and cell size, so lookup cannot hide an obstacle between grid centres.
        """
        resolution, step = 0.01, 0.02
        extent = horizon + length + margin + 0.1
        xmin, ymin = -extent, -extent
        size = int(np.ceil(2 * extent / resolution)) + 1
        maps = np.full((15, size, size), horizon, dtype=np.float32)
        corner_radius = np.hypot(max(offset, length - offset), width / 2)
        for index, steering in enumerate(np.linspace(-max_steer, max_steer, 15)):
            curvature = np.tan(steering * response) / wheelbase
            padding = (resolution / np.sqrt(2)
                       + step / 2 * (1 + abs(curvature) * corner_radius))
            inflated = margin + padding
            for distance in np.arange(step / 2, horizon, step):
                yaw = distance * curvature
                co, si = np.cos(yaw), np.sin(yaw)
                if abs(curvature) < 1e-9:
                    px, py = distance, 0.0
                else:
                    px = np.sin(yaw) / curvature + offset * (co - 1)
                    py = (1 - co) / curvature + offset * si
                cx, cy = px - length / 2 * co, py - length / 2 * si
                ex = (length / 2 + inflated) * abs(co) + (width / 2 + inflated) * abs(si)
                ey = (length / 2 + inflated) * abs(si) + (width / 2 + inflated) * abs(co)
                ix0 = max(0, int(np.floor((cx - ex - xmin) / resolution)))
                ix1 = min(size, int(np.ceil((cx + ex - xmin) / resolution)) + 1)
                iy0 = max(0, int(np.floor((cy - ey - ymin) / resolution)))
                iy1 = min(size, int(np.ceil((cy + ey - ymin) / resolution)) + 1)
                dx = xmin + np.arange(ix0, ix1)[None, :] * resolution - px
                dy = ymin + np.arange(iy0, iy1)[:, None] * resolution - py
                local_x, local_y = dx * co + dy * si, -dx * si + dy * co
                hit = ((local_x <= inflated) & (local_x >= -length - inflated)
                       & (np.abs(local_y) <= width / 2 + inflated))
                region = maps[index, iy0:iy1, ix0:ix1]
                region[hit] = np.minimum(region[hit], max(0.0, distance - step / 2))
        maps.setflags(write=False)
        return maps, xmin, ymin, resolution

    def _candidate_clearances(self, ranges, angles):
        """Evaluate all steering options, including the previous-command delay."""
        p = self.params
        maps, xmin, ymin, resolution = self._grid
        horizon = min(p.path_horizon, p.max_lidar_dist)
        delay = min(horizon, self.previous_speed * p.reaction_time)
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        if delay > 0:
            reaction_clearance = self._path_clearance(ranges, angles, self.previous_steering)
            if reaction_clearance <= delay:
                return np.full(len(self._steerings), reaction_clearance)
            curvature = np.tan(self.previous_steering * p.steering_response) / p.wheelbase
            ax, ay, yaw = self._arc(np.array([delay]), curvature)
            co, si = float(np.cos(yaw[0])), float(np.sin(yaw[0]))
            px = float(ax[0]) + p.lidar_offset * (co - 1)
            py = float(ay[0]) + p.lidar_offset * si
            x, y = (x - px) * co + (y - py) * si, -(x - px) * si + (y - py) * co
        ix = np.rint((x - xmin) / resolution).astype(int)
        iy = np.rint((y - ymin) / resolution).astype(int)
        valid = (ix >= 0) & (iy >= 0) & (ix < maps.shape[2]) & (iy < maps.shape[1])
        if not np.any(valid):
            return np.full(len(self._steerings), horizon)
        return np.minimum(horizon, delay + maps[:, iy[valid], ix[valid]].min(axis=1))

    def _front_blocked(self, ranges, angles):
        p = self.params
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        return bool(np.any((x >= 0) & (x <= p.emergency_distance + 1e-9)
                           & (np.abs(y) <= p.vehicle_width / 2)))

    def _speed_for_clearance(self, clearance):
        p = self.params
        available = max(0.0, clearance - p.stop_distance)
        if available <= 1e-9:
            return 0.0
        at = p.braking_deceleration * p.reaction_time
        return float(max(0.0, np.sqrt(at * at + 2 * p.braking_deceleration * available) - at))

    def _clearance_speed(self, ranges, angles, steering_angle):
        """Return the braking-limited speed for one forward steering path."""
        if self._front_blocked(ranges, angles):
            return 0.0
        return self._speed_for_clearance(self._path_clearance(ranges, angles, steering_angle))

    def _select_speed(self, steering_angle):
        p = self.params
        if abs(steering_angle) > p.straights_steering_angle:
            return p.corners_speed
        if abs(steering_angle) > p.fast_steering_angle:
            return p.straights_speed
        return p.fast_speed

    def _gaps(self, ranges):
        free = np.asarray(ranges) > 0
        edges = np.diff(np.r_[False, free, False].astype(int))
        return [(int(a), int(b)) for a, b in zip(np.flatnonzero(edges == 1),
                                               np.flatnonzero(edges == -1))
                if b - a > self.params.safe_threshold]

    def find_max_gap(self, free_space_ranges):
        """Return the widest qualifying gap, or (None, None) if none fits."""
        return max(self._gaps(free_space_ranges), key=lambda gap: gap[1] - gap[0],
                   default=(None, None))

    def find_best_point(self, start_i, end_i, ranges):
        """
        Return the index of the best aim point within ``[start_i, end_i)``.

        Uses a sliding-window average so the target favours the deepest part of
        the gap rather than a single noisy far reading. Small gaps just aim at
        the midpoint. Ties are resolved toward the gap midpoint.
        """
        window_size = min(
            self.params.best_point_conv_size, end_i - start_i)
        if window_size <= 1:
            return start_i + (end_i - start_i) // 2

        window = np.ones(window_size) / window_size
        averaged = np.convolve(
            ranges[start_i:end_i], window, 'valid')
        best_windows = np.flatnonzero(
            np.isclose(averaged, averaged.max()))
        target_midpoint = (end_i - start_i - 1) // 2
        target_indices = best_windows + (window_size - 1) // 2
        best_window = best_windows[
            np.argmin(np.abs(target_indices - target_midpoint))]
        return int(start_i + best_window + (window_size - 1) // 2)
