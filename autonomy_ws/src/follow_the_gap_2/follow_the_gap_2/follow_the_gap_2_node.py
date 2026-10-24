#!/usr/bin/env python3
import rclpy
from autonomy_core.base_drive_node import BaseDriveNode
import numpy as np

DEFAULT_PARAMS = {'bubble_radius': 0.4, 'preprocess_conv_size': 3, 'max_lidar_range': 5.0, 'speed_min': 1.0, 'speed_max': 3.0, 'steering_gain': 0.67, 'prev_steering_angle': 0.0, 'count': 0.0, 'max_steer': 0.60}
DEFAULT_PARAMS.update({
    'disparity_threshold': 0.7,  # adjacent range jump, metres
    'vehicle_width': 0.31,      # Gym vehicle width, metres
    'disparity_margin': 0.30,   # extra clearance on each side, metres
})

class ReactiveFollowGap(BaseDriveNode):
    """
    Implement the Follow the Gap reactive obstacle avoidance algorithm.
    Overview of the algorithm:
    1. Preprocess the LiDAR scan (smooth, clamp, handle bad values)
    2. Find the closest obstacle point
    3. Zero out a safety "bubble" around that closest point
    4. Find the longest consecutive run of non-zero (free) space — the
    "max gap"
    5. Pick the best steering target inside that gap
    6. Publish a drive command toward that target
    """
    def __init__(self):
        super().__init__('follow_the_gap_2_node', DEFAULT_PARAMS,
                         uses_odom=False, uses_camera=False, uses_lidar=True)
        for name in DEFAULT_PARAMS:
            setattr(self, name, getattr(self.params, name))

    def scan_callback(self, scan_msg):
        ranges = np.asarray(scan_msg.ranges)
        if (ranges.size < 2 or np.count_nonzero(np.isfinite(ranges)) == 0
                or not np.isfinite(scan_msg.angle_increment)
                or scan_msg.angle_increment <= 0):
            self._warn_rate_limited(
                'No valid LiDAR input received. Publishing stop command.')
            self.publish_stop()
            return

        result = self.compute_drive(scan_msg)
        if result is None:
            self.publish_stop()
            return

        steering_angle, speed = result
        self.publish_drive(steering_angle, speed)
        self._log_status(speed, steering_angle)


    def preprocess_lidar(self, ranges):
        
        """
        Preprocess the raw LiDAR scan array before running the algorithm.
        Steps to implement:
        1. Replace NaN / Inf values with safe numbers
        2. Clamp all values to [0, max_lidar_range]
        3. Apply a moving-average (convolution) smoothing filter
        Args:
        ranges: 1-D numpy array of raw LiDAR distances (meters)
        Returns:
        proc_ranges: cleaned and smoothed 1-D numpy array
        """
        
        #Converting the input to a numpy float array.
        
        proc_ranges = np.array(ranges, dtype=float) 
        
        # TODO (2/3): Replace NaN and ±Inf values with safe defaults.
        # NaN / +Inf → self.max_lidar_range (treat as "very far away")
        # -Inf → 0.0 (treat as "right on top of us")
        # Then clamp every value into [0, self.max_lidar_range].
        # Hint: np.nan_to_num(), np.clip()

        proc_ranges[(np.isnan(proc_ranges))] = self.max_lidar_range
        proc_ranges[(np.isposinf(proc_ranges))] = self.max_lidar_range
        proc_ranges[(np.isneginf(proc_ranges))] = 0.0
        
        proc_ranges= np.clip(proc_ranges, 0.0, self.max_lidar_range)

        # TODO (3/3): Smooth the array with a moving-average filter.
        # Build a kernel of length self.preprocess_conv_size filled with
        # (1 / self.preprocess_conv_size) and convolve with proc_ranges.
        # Only apply smoothing when self.preprocess_conv_size > 1.
        # Hint: np.ones(), np.convolve(..., mode='same')
        
        if self.preprocess_conv_size>1:    
            window = min(self.preprocess_conv_size, len(proc_ranges))
            kernel= np.ones(window)
            kernel = 1/(window*kernel)

            proc_ranges= np.convolve(proc_ranges, kernel, mode='same')

        return proc_ranges
        # ------------------------------------------------------------------
    
    def extend_disparities(self, processed_ranges, raw_ranges, angle_increment):
        """Extend near corner edges into their farther side by car half-width.

        Detect edges before smoothing, which can hide a sharp distance jump.
        Use the original distances for every edge so extensions do not create
        further artificial edges. Only reduce ranges, preserving closer walls.
        """
        extended = processed_ranges.copy()
        distances = np.clip(np.nan_to_num(
            np.asarray(raw_ranges, dtype=float), nan=self.max_lidar_range,
            posinf=self.max_lidar_range, neginf=0.0),
            0.0, self.max_lidar_range)
        clearance = max(0.0, self.vehicle_width / 2.0 + self.disparity_margin)
        if distances.size < 2 or clearance == 0.0:
            return extended

        edges = np.flatnonzero(
            np.abs(np.diff(distances)) > self.disparity_threshold)
        for edge in edges:
            if distances[edge] < distances[edge + 1]:
                near_idx, direction = edge, 1
            else:
                near_idx, direction = edge + 1, -1
            near_distance = distances[near_idx]
            half_angle = np.arcsin(min(1.0, clearance / max(near_distance, 1e-6)))
            beams = min(len(extended), int(np.ceil(half_angle / angle_increment)))
            if direction == 1:
                affected = slice(near_idx, min(len(extended), near_idx + beams + 1))
            else:
                affected = slice(max(0, near_idx - beams), near_idx + 1)
            extended[affected] = np.minimum(extended[affected], near_distance)
        return extended

    def find_max_gap(self, free_space_ranges):
        
        """
        Find the largest contiguous "gap" (run of non-zero values) in
        free_space_ranges. Zeros represent obstacle / bubble regions.
        Args:
        free_space_ranges: 1-D numpy array where 0 = blocked, >0 = free
        Returns:
        start_idx: first index of the longest free run
        end_idx: last index of the longest free run (inclusive)
        """
        # TODO (1/4): Build a boolean mask where True means the cell is free
        # (i.e., the value is greater than 0).
        # Hint: mask = free_space_ranges > 0

        mask = free_space_ranges>0 # replace with your implementation

        # TODO (2/4): Find the start and end boundaries of every run of Trues.
        # Strategy — pad the mask with a 0 on each side, compute np.diff,
        # then look for +1 transitions (gap starts) and -1 transitions (gap ends).
        # Hint: np.concatenate, np.diff, mask.astype(int), np.where
        
        padded = np.concatenate(([0], mask.astype(int), [0]))
        diff = np.diff(padded)
        
        starts = np.where(diff == 1)[0]   # inclusive
        ends = np.where(diff == -1)[0]    # exclusive

        # TODO (3/4): Handle the edge case where no free space exists.
        # Log a warning and return (0, 0) so the caller can detect failure.
        # (3) No free space at all
        if len(starts) == 0:
            self.get_logger().warning("No free gap found.")
            return None, None
        
        # TODO (4/4): Compute the length of each gap (ends - starts), find the
        # index of the maximum, and return the corresponding start and
        # (end - 1) so that end_idx is inclusive.
        # Hint: np.argmax()
        lengths = ends - starts
        max_gap_idx = np.argmax(lengths)

        start_idx = starts[max_gap_idx]
        end_idx = ends[max_gap_idx] - 1   # convert exclusive to inclusive
       
        return start_idx, end_idx # replace with your computed start_idx, end_idx
        # ------------------------------------------------------------------
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        Choose the single best index inside the gap [start_i, end_i] to
        steer toward.
        Three simple strategies (pick one, or blend them):
        4. Furthest point — index of maximum range inside the gap
        5. Centered point — midpoint index of the gap
        6. Weighted blend — e.g. 80 % furthest + 20 % centered
        
        Args:
        start_i: first index of the gap
        end_i: last index of the gap (inclusive)
        ranges: full processed LiDAR array
        Returns:
        best_idx: the index in `ranges` to aim for
        """
        
        # TODO (1/4): Guard against degenerate inputs.
        # If start_i > end_i or end_i is out of bounds, return the
        # center index of the full array as a safe fallback.
        # Hint: len(ranges) // 2

        if (np.isnan(start_i) or np.isnan(end_i) or np.isinf(start_i) or np.isinf(end_i) or start_i > end_i or start_i < 0 or end_i >= len(ranges)):
            return len(ranges)//2

        # TODO (2/4): Slice out just the gap portion of the ranges array.
        gap_ranges = ranges[start_i : end_i + 1]
        
        max_indices = np.flatnonzero(gap_ranges == np.max(gap_ranges))
        groups = np.split(
            max_indices,
            np.where(np.diff(max_indices) > 1)[0] + 1
        )

        longest = max(groups, key=len)

        # TODO (3/4): Compute the three candidate indices described above.
        furthest_idx = start_i + (longest[0] + longest[-1]) // 2
        centered_idx = (start_i + end_i) // 2
        blended_idx = int(0.8 * furthest_idx + 0.2 * centered_idx)
        # TODO (4/4): Return your chosen best_idx.
        # Start with the naive furthest-point strategy, then experiment
        # with centering or blending to see what drives best.
        return blended_idx # replace with your implementation
        # ------------------------------------------------------------------
    
    def compute_drive(self, data):
        
        """
        Main callback: runs the full Follow the Gap pipeline on every
        incoming LiDAR scan and publishes a drive command.
        Args:
        data: sensor_msgs/LaserScan message
        """
        ranges = np.array(data.ranges)
        if (ranges.size < 2 or not np.isfinite(data.angle_increment)
                or data.angle_increment <= 0):
            return None

        # Step 1: Preprocess — smooth, clamp, fix bad values
        proc_ranges = self.preprocess_lidar(ranges)
        
        n = len(proc_ranges)
        #angles = data.angle_min + np.arange(n) * data.angle_increment

        # Keep everything except the rear sector
        #front_mask = (angles >= -np.deg2rad(115)) & (angles <= np.deg2rad(115))
          
        #valid_indices = np.where(front_mask)[0]
        #proc_ranges_front = proc_ranges[valid_indices]
        #proc_ranges_copy = proc_ranges_front.copy()
        n=len(proc_ranges)

        start = n // 5
        end = 4 * n // 5
        proc_ranges_front= proc_ranges[start:end]
        len_front= len(proc_ranges_front)
        mid= len_front//2

        # Width of the center window to inspect
        center_half_width = 100   # tune this, try 60 to 100

        center_start = max(0, mid - center_half_width)
        center_end = min(len_front, mid + center_half_width)

        center_slice = proc_ranges_front[center_start:center_end]
        
        # Temporary debug: compare one raw value to its processed counterpart
        self.get_logger().info( f'Raw[0]: {ranges[0]:.2f} | Processed[0]: {proc_ranges[0]:.2f}', throttle_duration_sec=2.0)
        # TODO: Step 2 — Find the closest point.
        # Make a copy of proc_ranges to modify (so you keep the original).
        # Isolate non-zero values, find the minimum distance and its index.
        # If there are NO non-zero values, log a warning, publish (0, 0),
        # and return early.
        # Hint: proc_ranges_copy = proc_ranges.copy()
        # nonzero = proc_ranges_copy[proc_ranges_copy > 0]
        # closest_distance = np.min(nonzero)
        # closest_idx = np.argmin(proc_ranges_copy)
        
        proc_ranges_copy=proc_ranges_front.copy()
        nonzero= proc_ranges_copy[proc_ranges_copy>0]
       
        if len(nonzero) == 0:
            self.get_logger().warning("No valid LiDAR points found. Stopping vehicle.")
            return None
        
        center_closest_dist = float("inf")
        valid = np.where(np.isfinite(center_slice))[0]

        if len(valid) > 0:
            local_idx = valid[np.argmin(center_slice[valid])]
            center_closest_idx = center_start + local_idx
            center_closest_dist = proc_ranges_front[center_closest_idx]

        if center_closest_dist < 0.4:   # tune threshold
            closest_idx = center_closest_idx
            closest_distance = center_closest_dist
 
        else:

            closest_distance = np.min(nonzero)
        # index of that closest point in the original processed array
            closest_idx = np.where(proc_ranges_copy == closest_distance)[0][0]

        # TODO: Step 3 — Eliminate all points inside the safety bubble.
        # Call self.get_bubble_indices() to get the affected indices, then/
        # set proc_ranges_copy at those indices to 0.0.
        # Hint: bubble_indices = self.get_bubble_indices(data, closest_idx, closest_distance)/
        # proc_ranges_copy[bubble_indices] = 0.0
        # Protect all detected corner edges before selecting the gap/target.
        # Keep the existing closest-obstacle selection and safety bubble.
        proc_ranges_copy = self.extend_disparities(
            proc_ranges_front, ranges[start:end], data.angle_increment)
        bubble_indices= self.get_bubble_indices(closest_idx, closest_distance, len(proc_ranges_copy), data.angle_increment)
        proc_ranges_copy[bubble_indices] = 0.0
        self.get_logger().info(f'Bubble: {len(bubble_indices)} points cleared around index {closest_idx}', throttle_duration_sec=1.0)

        # TODO: Step 4 — Find the max-length gap.
        # Call self.find_max_gap(proc_ranges_copy).
        # If both returned indices are 0 (no valid gap), log a warning,
        # publish a slow forward command, and return early.
        start_i, end_i = self.find_max_gap(proc_ranges_copy)

        if start_i is None:
            self.get_logger().warning("No valid gap found. Publishing slow forward command.")
            return float(self.prev_steering_angle), 1

        gap_length = end_i - start_i + 1
        self.get_logger().info(f'Max gap: start={start_i}, end={end_i}, length={gap_length}',throttle_duration_sec=1.0)
        # TODO: Step 5 — Find the best point inside that gap.
        best_idx=self.find_best_point(start_i, end_i, proc_ranges_copy)
        global_best_idx = start + best_idx
        #global_best_idx = valid_indices[best_idx]

        # TODO: Step 6 — Convert best_idx to a steering angle and publish.
        angle_to_best = data.angle_min + global_best_idx *data.angle_increment

        #abs_raw = self.steering_gain * angle_to_best

        steering_angle =self.steering_gain * angle_to_best

        # Clamp to ± np.deg2rad(30.0) for safety.
        max_steering = self.max_steer
        steering_angle= np.clip(steering_angle, -max_steering, max_steering)
        abs_raw= abs(steering_angle)
        
        if abs_raw<= np.deg2rad(8):
            steering_angle= 0.58 *self.prev_steering_angle + 0.42 *steering_angle
        else:
            steering_angle= 0.20 *self.prev_steering_angle + 0.80 *steering_angle
        #steering_angle=steering_angle   
        steering_angle = float(np.clip(steering_angle, -max_steering, max_steering))
        self.prev_steering_angle= steering_angle

        # TODO: Step 7 — Compute speed based on steering magnitude and gap size.
        # Larger steering angle → slower speed.
        # Use thresholds like 10° and 20° (convert with np.deg2rad) to
        # choose between speed_max, average, and speed_min.
        # Optionally further reduce speed if the gap is narrower than
        # ~50 indices (a "min_safe_gap" threshold).
        gap_length = end_i - start_i + 1
        abs_steer = abs(steering_angle)

        small_turn = np.deg2rad(5.0)   
        medium_turn = np.deg2rad(15.0)
        sharp_turn = np.deg2rad(20.0)
        min_safe_gap = 60

        if abs_steer < small_turn:
            speed = self.speed_max
            self.count=self.count+1
        elif abs_steer < medium_turn:
            speed = 2.2
        elif abs_steer < sharp_turn:
            speed = 1.5

        else:
            speed = self.speed_min
        
        #if self.count >=3:
         #   speed= 0.05
          #  self.count=0
        if gap_length < 100:
            speed= 1.5
        if gap_length < min_safe_gap:
            speed = self.speed_min

        # Apply the configured limit after every speed-selection branch.
        speed = min(speed, self.speed_max)

        # TODO: Step 8 — Publish the drive command.
        # Call self.publish_drive(steering_angle, speed).
        # Also log the gap bounds, best index, angle, and speed for debugging.

        self.get_logger().info( f'Best idx={best_idx}, angle={np.rad2deg(angle_to_best):.1f} deg, 'f'steer={np.rad2deg(steering_angle):.1f} deg, speed={speed:.2f}',
        throttle_duration_sec=1.0)
        return float(steering_angle), float(speed)

        # ======================================================================
        # Helper methods — implement these alongside the steps above
        
        # ======================================================================
    def get_bubble_indices(self, closest_idx, closest_distance, scan_length, angle_increment):
        """
        Return every LiDAR index whose angle falls within the safety bubble
        centered on closest_idx.
        The bubble subtends an angle of approximately
        bubble_angle ≈ self.bubble_radius / closest_distance (radians)
        which converts to an index half-width of
        bubble_idx_range = ceil(bubble_angle / data.angle_increment)
        Args:
        data: LaserScan message (provides angle_increment
        and
        total number of beams)
        closest_idx: index of the nearest obstacle point
        closest_distance: distance to that point (meters)
        Returns:
        bubble_indices: 1-D numpy array of integer indices to zero out
        """
        # TODO (1/3): Guard against division by zero — if closest_distance
        if closest_distance<=0:
            closest_distance=0.1
        # is 0, set it to a small positive value (e.g. 0.1).
        # TODO (2/3): Calculate the angular half-width of the bubble and
        # convert it to an index count.
        bubble_angle = self.bubble_radius / closest_distance
        bubble_idx_range = int(np.ceil(bubble_angle / angle_increment))
    
        # TODO (3/3): Compute start and end indices (clamped to valid range)
        # and return np.arange(start_idx, end_idx + 1).
        # Hint: max(0, ...) and min(len(data.ranges) - 1, ...)
        start_idx= max(0, closest_idx-bubble_idx_range)
        end_idx= min(scan_length-1, closest_idx+bubble_idx_range)
    
        return np.arange(start_idx, end_idx+1) # replace with your implementation

def main(args=None):
    
    rclpy.init(args=args)
    print("Follow the Gap Initialized")
    reactive_node = ReactiveFollowGap()
    
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
         rclpy.shutdown()
    
if __name__ == '__main__':
    
    main()
