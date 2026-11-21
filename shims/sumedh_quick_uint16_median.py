"""
HWEMU stand-in for sumedh-quick-uint16-median, whose only build is a Jetson (aarch64) wheel.

median(depth, max_invalid) is what otto main replaced it with (project_otto/vision/
target_detector/_target_pose_estimator.py, otto 36494ace): the median of the nonzero
(valid) depths, by np.partition (at most one index off, as there), or NaN when there are
none or more than `max_invalid` of them (a fraction) are zero -- the C++ module's NaN,
which the 2026 comp branch's caller checks for.
"""
import numpy as np


def median(depth, max_invalid):
    flat = np.asarray(depth).ravel()
    if flat.size == 0:
        return float("nan")
    valid = flat[flat != 0]
    if valid.size == 0 or 1 - (valid.size / flat.size) > max_invalid:
        return float("nan")
    mid = valid.size // 2
    return float(np.partition(valid, mid)[mid])
