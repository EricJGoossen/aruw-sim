# HWEMU stand-in for rm-pose-solver, whose only build is a Jetson (aarch64) wheel.
# Below, unchanged, is otto main's own Python port of it (project_otto/vision/
# target_detector/_yaw_solver.py at otto 36494ace). run.sh puts shims/ on otto's
# PYTHONPATH, so a branch that still imports the wheel (the 2026 comp branch) gets
# this on x86.

"""
Plate yaw solver: finds the plate yaw that best reprojects the plate corners onto the image.

Python port of `solve_yaw` from rm-pose-solver (https://github.com/SumedhP/robomaster-yaw-correction).
The C++ version ran the two bounded searches on a thread pool with an optimized build. This port
uses scipy and is called up to twice per detected plate, so it is a potential bottleneck in the
detection pipeline; profile it before adding more callers or calls per frame.
"""

from typing import Any, List, Tuple

import numpy as np
import numpy.typing as npt
from scipy.optimize import minimize_scalar  # type: ignore

Tensor = npt.NDArray[np.float64]

# Yaw tolerance of each bounded search
_YAW_TOLERANCE_RAD = np.radians(0.1)

# Number of evenly spaced samples used to locate the split point between the two searches
_NUM_COARSE_SAMPLES = 20

_MAX_ITERATIONS = 200


def _build_yx_rotation(pitch: float, roll: float) -> Tensor:
    """Rotation about Y by `pitch` followed by rotation about X by `roll` (Ry @ Rx)."""
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    return np.array(
        [
            [cp, sp * sr, sp * cr],
            [0.0, cr, -sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _squared_reprojection_error(
    yaw: float,
    world_to_camera_rotation: Tensor,
    plate_yx_rotation: Tensor,
    plate_points: Tensor,
    position: Tensor,
    camera_matrix: Tensor,
    image_points: Tensor,
) -> float:
    """Sum of squared pixel errors of the 4 projected plate corners for a given yaw."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    yaw_rotation = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation = world_to_camera_rotation @ yaw_rotation @ plate_yx_rotation

    # Corners in camera frame, shape (3, 4); X is forward, Y left, Z up
    corners = rotation @ plate_points.T + position[:, np.newaxis]
    inv_x = 1.0 / corners[0]
    u = camera_matrix[0, 0] * (-corners[1] * inv_x) + camera_matrix[0, 2]
    v = camera_matrix[1, 1] * (-corners[2] * inv_x) + camera_matrix[1, 2]

    du = u - image_points[:, 0]
    dv = v - image_points[:, 1]
    return float(np.sum(du * du + dv * dv))


def solve_yaw(
    image_points: Tensor,
    position_xyz: Tensor,
    plate_matrix: Tensor,
    camera_matrix: Tensor,
    plate_pitch: float,
    plate_roll: float,
    cam_pitch: float,
    cam_roll: float,
    yaw_lo: float,
    yaw_hi: float,
) -> Tuple[float, float, float, float]:
    """
    Finds the two locally optimal plate yaws within [yaw_lo, yaw_hi].

    The reprojection error over yaw typically has two minima (the true yaw and its mirror), so
    the range is split at the first local maximum of a coarse sweep and each half is searched
    separately.

    Args:
        image_points: Observed plate corners in pixels, shape (4, 2).
        position_xyz: Plate center in camera frame, shape (3,).
        plate_matrix: Plate corners in plate frame, shape (4, 3).
        camera_matrix: Camera intrinsics, shape (3, 3).
        plate_pitch: Plate pitch in radians.
        plate_roll: Plate roll in radians.
        cam_pitch: Camera pitch in radians.
        cam_roll: Camera roll in radians.
        yaw_lo: Lower yaw bound in radians.
        yaw_hi: Upper yaw bound in radians.
    Returns:
        (yaw1, rms_error_px1, yaw2, rms_error_px2): best yaw and RMS corner reprojection error
        for the lower and upper halves of the range.
    """
    if not (np.isfinite(yaw_lo) and np.isfinite(yaw_hi)):
        raise ValueError("yaw bounds must be finite")
    if yaw_hi < yaw_lo:
        raise ValueError("yaw_hi must be >= yaw_lo")
    if image_points.shape != (4, 2):
        raise ValueError("image_points must be shape (4, 2)")
    if position_xyz.shape != (3,):
        raise ValueError("position_xyz must be shape (3,)")
    if plate_matrix.shape != (4, 3):
        raise ValueError("plate_matrix must be shape (4, 3)")
    if camera_matrix.shape != (3, 3):
        raise ValueError("camera_matrix must be shape (3, 3)")

    args = (
        _build_yx_rotation(cam_pitch, cam_roll).T,
        _build_yx_rotation(plate_pitch, plate_roll),
        plate_matrix,
        position_xyz,
        camera_matrix,
        image_points,
    )

    def cost(yaw: float) -> float:
        return _squared_reprojection_error(yaw, *args)

    # Split at the first local maximum of a coarse sweep; default to the midpoint
    step = (yaw_hi - yaw_lo) / _NUM_COARSE_SAMPLES
    mid = 0.5 * (yaw_lo + yaw_hi)
    prev_cost = None
    for i in range(1, _NUM_COARSE_SAMPLES):
        yaw = yaw_lo + i * step
        current_cost = cost(yaw)
        if prev_cost is not None and current_cost > prev_cost and cost(yaw + step) < current_cost:
            mid = yaw
            break
        prev_cost = current_cost

    results: List[Tuple[float, float]] = []
    for lo, hi in ((yaw_lo, mid), (mid, yaw_hi)):
        result: Any = minimize_scalar(  # pyright: ignore[reportUnknownVariableType]
            cost,
            bounds=(lo, hi),
            method="bounded",
            options={"xatol": _YAW_TOLERANCE_RAD, "maxiter": _MAX_ITERATIONS},
        )
        # RMS over the 4 corners
        results.append((float(result.x), float(np.sqrt(0.25 * result.fun))))

    (yaw1, error1), (yaw2, error2) = results
    return yaw1, error1, yaw2, error2
