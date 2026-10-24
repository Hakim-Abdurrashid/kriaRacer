"""
Follow-the-Gap reactive planner (pure Python, no ROS).

This module holds the gap-following *algorithm* only. It takes a raw range
array plus the scan geometry and returns a ``(steering_angle, speed)`` command.
Keeping it free of ROS means it can be unit-tested with synthetic scans (see
``test/test_planner.py``) and reused by any node.

The pipeline mirrors the classic RoboRacer "Follow the Gap" / Disparity Extender:

1. clamp ranges and crop to the forward field of view,
2. smooth to suppress single-beam noise,
3. zero out a safety "bubble" around the closest obstacle,
4. find the widest remaining gap and aim at the best point inside it,
5. pick a speed from the steering magnitude (slower in corners).

"""

import numpy as np

#from autonomy_core import lidar_utils


class FollowTheGapPlanner:
    """Stateless gap-following planner driven by a parameter namespace."""

    def __init__(self, params):
        """
        Store the tuning parameters.

        :param params: an object (e.g. ``argparse.Namespace``) exposing the
            follow-the-gap tunables as attributes — see ``config/params.yaml``.
        """
        self.params = params


    def plan(self, mask):
        """Turn one drivable-area mask into a (steering_angle, speed) command."""
        column_ranges = self.gap_follow_algorithm(mask)
        steering_angle = self.find_max_gap(column_ranges, self.params.min_clearance)

        if steering_angle is None:
            return None  # BaseDriveNode treats this as a stop request

        speed = self._select_speed(steering_angle)
        return steering_angle, speed

    def gap_follow_algorithm(self, mask, row_start=None, row_end=None):

        h, w= mask.shape

        if row_end is None:
            row_end=h #bottom of the image

        if row_start is None:
            row_start=0 #top of the image


        roi= mask[row_start:row_end, :]

        roi_h= roi.shape[0] #number of rows is equal to height

        column_ranges=np.zeros(w, dtype=np.float32)

        for col in range(w):
            column=roi[:, col] #all rows of the col
            drivable_row= np.flatnonzero(column)

            if drivable_row.size==0:
                column_ranges[col]=0.0 #fully blocked
                continue

            bottom_row=roi_h-1 #since image's height starts from below

            if column[bottom_row]==0:
                column_ranges[col]=0.0 #blocked from the start (an obstacle)
                continue

            r=bottom_row

            while r>=0 and column[r]!=0:

                r-=1

                clearance= bottom_row-r
                column_ranges[col]=clearance

        return column_ranges

    def _select_speed(self, steering_angle):
        """Pick a speed band from the steering magnitude (slower in corners)."""
        p = self.params
        if abs(steering_angle) > p.straights_steering_angle:
            return p.corners_speed
        if abs(steering_angle) > p.fast_steering_angle:
            return p.straights_speed
        return p.fast_speed

    def find_max_gap(self, free_space_ranges, min_clearance=10):
        """
        Return the (start, stop) indices of the widest non-zero gap.

        Prefers the longest run of free space that also clears the configured
        ``safe_threshold``; falls back to the longest run otherwise. Returns
        ``(None, None)`` when there is no free space at all.
        """

        w=len(free_space_ranges)

        free= free_space_ranges>=min_clearance

        best_start, best_len=0, 0
        cur_start, cur_len=None, 0

        for i, f in enumerate(free):

            if f:
                if cur_start is None:
                    cur_start=i

                cur_len+=1
                if cur_len> best_len:
                    best_len=cur_len
                    best_start=cur_start

            else:
             cur_start, cur_len=None, 0

        if best_len==0:
            return None


        #target column
        gap_slice=free_space_ranges[best_start: best_start+best_len]
        best_col=np.argmax(gap_slice)
        target_col= best_start+best_col


        #steering
        center_col=w/2.0
        max_steer_rad= self.params.max_steer
        normalized_offset= (target_col-center_col)/center_col
        steering_angle=normalized_offset*max_steer_rad
        
        return steering_angle

   
