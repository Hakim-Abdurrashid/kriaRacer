"""
Reusable, ROS-free LiDAR processing helpers.

Every reactive planner in this workspace ends up doing the same handful of
operations on a ``sensor_msgs/LaserScan``: throw away ranges outside the
forward field of view, smooth out noise, clamp impossible distances, and
convert a beam index back into a steering angle.

These functions take and return plain NumPy arrays / scalars so they can be
unit-tested without spinning up a ROS node. The angle math is driven by the
scan's *real* ``angle_min`` / ``angle_increment`` rather than assuming a full
360 deg sweep, so it is correct for any LiDAR (the Hokuyo on this car only
covers ~270 deg).
"""

import numpy as np


def crop_to_fov(ranges, angle_min, angle_increment, fov_rad):
    """
    Keep only the beams within a forward field of view centred on 0 rad.

    Returns the cropped range array together with the bearing of its first
    element, so callers can later map an index back to an absolute angle.

    :param ranges: 1-D array of range readings, beam 0 first.
    :param angle_min: bearing of ``ranges[0]`` in radians (usually negative).
    :param angle_increment: angular step between consecutive beams, in radians.
    :param fov_rad: total forward field of view to keep, in radians. The kept
        window is ``[-fov_rad / 2, +fov_rad / 2]`` relative to straight ahead.
    :returns: ``(cropped_ranges, angle_of_first)`` where ``angle_of_first`` is
        the bearing of ``cropped_ranges[0]``.
    """
    half_fov = fov_rad / 2.0

    # Convert the desired angular window into array indices. The readings are
    # ordered by increasing bearing, so the window maps to a contiguous slice.
    start = int(np.ceil((-half_fov - angle_min) / angle_increment))
    stop = int(np.floor((half_fov - angle_min) / angle_increment)) + 1

    # Clamp to the array bounds in case the requested FOV is wider than the
    # sensor actually provides.
    start = max(start, 0)
    stop = min(stop, len(ranges))

    angle_of_first = angle_min + start * angle_increment
    return ranges[start:stop], angle_of_first


def smooth(ranges, window):
    """
    Return a moving-average smoothed copy of ``ranges``.

    A small window suppresses single-beam noise without blurring real gaps.
    Edge values are extended while filtering so scan boundaries are not
    artificially shortened. ``window`` of 1 (or less) returns the input
    unchanged.
    """
    ranges = np.asarray(ranges, dtype=float)
    if window <= 1 or ranges.size == 0:
        return ranges.copy()

    left = (window - 1) // 2
    right = window - 1 - left
    padded = np.pad(ranges, (left, right), mode='edge')
    kernel = np.ones(window) / window
    return np.convolve(padded, kernel, 'valid')


def clip_ranges(ranges, max_dist):
    """
    Clamp ranges into ``[0, max_dist]``.

    Saturating far-away and infinite readings keeps the gap-finding logic from
    chasing noisy long-range returns and removes ``inf`` values produced when a
    beam sees nothing.
    """
    return np.clip(ranges, 0.0, max_dist)


def index_to_steering(index, angle_of_first, angle_increment, max_steer, gain=1.25):
                      #gain=0.5):
    """
    Convert a beam index into a steering angle command.

    The true bearing of the target beam is ``angle_of_first + index *
    angle_increment`` (positive = left, matching the ROS LaserScan convention).
    That bearing is scaled by ``gain`` to damp the response and then clamped to
    the vehicle's mechanical steering limit.

    :param index: index into the cropped range array of the chosen target beam.
    :param angle_of_first: bearing of element 0 of the cropped array, in rad.
    :param angle_increment: angular step between beams, in radians.
    :param max_steer: maximum absolute steering angle, in radians.
    :param gain: proportional gain applied to the target bearing.
    :returns: steering angle in radians, clamped to ``[-max_steer, max_steer]``.
    """
    bearing = angle_of_first + index * angle_increment
    steering_angle = bearing * gain
    return float(np.clip(steering_angle, -max_steer, max_steer))
