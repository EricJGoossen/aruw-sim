"""
Hands otto the enemy's true state, so its auto-nav (and aim) react to an enemy in the sim.

otto finds enemies through its armor detector, which is CUDA/TensorRT-only and can't run here
(shims/hwemu_detector_stub.py stands in and detects nothing), then tracks the selected one
with a particle filter (a Jetson-only wheel on the 2026 comp branch). So in a sim run otto
never has a target, and its motion strategy never chases or reacts. With HWEMU_TRUTH_TARGETS
set (scripts/run.sh --enemy sets it), sitecustomize wraps otto's Turret.update_selector with
`install` below: while an enemy is in sight, each turret's selected robot becomes the enemy robot the hub moves (hw-emulator actors), read from the newest row of
the hub's truth log (<runtime dir>/truth.jsonl, flushed row by row), and its "filter" just
returns that state; otherwise otto's own selection runs (and, with no detections, finds
nothing). Everything downstream -- target selection's result, the motion
strategy's attack/chase, the aim message to the MCB -- is otto's own code.

What otto sees is gated, roughly as a camera would see it: an enemy within
HWEMU_TRUTH_TARGET_RANGE metres (default 8) of our chassis, with no field wall at least 0.3 m
tall (one a camera ~0.45 m up can't see a plate 0.16 m up over) between the two, in 2-D. The
turret's field of view isn't modelled: once in line of sight, it's seen. Reported as the sim
gap `truth_targets: enabled`, so a run never looks like working vision.

The world frame is the field's (red's): otto's world frame is the MCB's odometry frame, which
starts at our spawn in field coordinates (assets/field/arc2026_3v3.yaml).
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

# The enemy sentry, from its model (build/assets/enemy): armor plates 0.30 m out from the
# centre, their centres 0.0627 + 0.10 m above the footprint.
PLATE_RADIUS = 0.30
PLATE_Z = 0.0627 + 0.10
ENEMY_ID_BASE = 9000  # otto's tracked-robot ids are small; these stay clear of them
SCORE = 0.05  # target_selector scores: lower is better (FireRate.HIGH below 2/5 of the threshold)
WALL_MIN_HEIGHT = 0.3


def _otto(*candidates: str) -> Any:
    """The first module that imports: the 2026 comp branch's path or main's (post-reorg)."""
    import importlib

    for name in candidates:
        try:
            return importlib.import_module(name)
        except ImportError:
            continue
    raise ImportError(f"none of {candidates}")


def _walls(repo: str) -> List[Tuple[float, float, float, float]]:
    """The field's walls tall enough to hide a plate, as 2-D boxes, the mirrored ones too."""
    import yaml

    with open(os.path.join(repo, "assets", "field", "arc2026_3v3.yaml")) as f:
        field = yaml.safe_load(f)
    mx = field["mirror_x"]
    out = []
    for el in field["elements"]:
        if "box" not in el or not el.get("collide", True):
            continue
        (x0, y0, _), (x1, y1, z1) = el["box"]
        if z1 < WALL_MIN_HEIGHT:
            continue
        out.append((x0, y0, x1, y1))
        if el.get("mirror"):
            out.append((2 * mx - x1, y0, 2 * mx - x0, y1))
    return out


def _segment_hits_box(a: Tuple[float, float], b: Tuple[float, float], box: Tuple[float, float, float, float]) -> bool:
    """Whether segment a-b crosses an axis-aligned box (Liang-Barsky)."""
    x0, y0, x1, y1 = box
    t0, t1 = 0.0, 1.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if p == 0:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            t0 = max(t0, r)
        else:
            t1 = min(t1, r)
        if t0 > t1:
            return False
    return True


class _Truth:
    """The newest row of the hub's truth log, and each actor's velocity from the last two."""

    def __init__(self, path: str):
        self._path = path
        self._last: Dict[str, Tuple[float, float, float, float]] = {}  # name -> t, x, y, yaw
        self._vel: Dict[str, Tuple[float, float, float]] = {}  # name -> vx, vy, omega

    def latest(self) -> Optional[Dict[str, Any]]:
        try:
            with open(self._path, "rb") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 8192))
                lines = f.read().splitlines()
        except OSError:
            return None
        for line in reversed(lines):
            try:
                row = json.loads(line)
                break
            except ValueError:
                continue  # the first line of the window may be cut
        else:
            return None
        for name, (x, y, yaw) in (row.get("actors") or {}).items():
            prev = self._last.get(name)
            t = row["t"]
            if prev is not None and t > prev[0]:
                dt = t - prev[0]
                dyaw = math.remainder(yaw - prev[3], 2 * math.pi)
                self._vel[name] = ((x - prev[1]) / dt, (y - prev[2]) / dt, dyaw / dt)
            if prev is None or t > prev[0]:
                self._last[name] = (t, x, y, yaw)
        return row

    def velocity(self, name: str) -> Tuple[float, float, float]:
        return self._vel.get(name, (0.0, 0.0, 0.0))


class _TruthFilter:
    """Stands in for otto's RobotParticleFilter: its estimate is the truth it was given."""

    is_initialized = True

    def __init__(self, robot: Any):
        self.robot = robot

    def initialize_filter(self, observer_pos: Any, robot: Any, timestamp: Any, filter_id: Any, prior: Any = None) -> None:
        self.robot = robot

    def update_filter(self, observer_pos: Any, robot: Any, timestamp: Any) -> None:
        self.robot = robot


def install(turret_module: Any, repo: str) -> None:
    """Wraps turret_module.Turret.update_selector (see the module docstring)."""
    import numpy as np

    from hwemu import gaps

    target = _otto("project_otto.target.target", "project_otto.vision.target.target")
    selector = _otto("project_otto.target_selector._scored_target", "project_otto.vision.target_selector._scored_target")
    spatial = _otto("project_otto.spatial", "project_otto.core.spatial")
    frames = _otto("project_otto.frames", "project_otto.core.frames")
    icons = _otto("project_otto.robomaster._icon_type", "project_otto.core.robomaster._icon_type")
    world = frames.WorldFrame

    truth = _Truth(os.path.join(os.environ["HWEMU_RUNTIME_DIR"], "truth.jsonl"))
    walls = _walls(repo)
    reach = float(os.environ.get("HWEMU_TRUTH_TARGET_RANGE", "8"))
    gaps.report(
        "truth_targets",
        "enabled",
        "otto's targets are the sim's true enemy (shims/hwemu_truth_targets.py), not detections: "
        f"any enemy within {reach:g} m in line of sight is selected, its state exact",
    )

    def seen_enemy() -> Optional[Any]:
        row = truth.latest()
        if row is None or not row.get("actors"):
            return None
        me = (row["x"], row["y"])
        best = None
        for i, (name, (x, y, yaw)) in enumerate(sorted(row["actors"].items())):
            d = math.hypot(x - me[0], y - me[1])
            if d > reach or any(_segment_hits_box(me, (x, y), w) for w in walls):
                continue
            if best is None or d < best[0]:
                best = (d, i, name, x, y, yaw)
        if best is None:
            return None
        _, i, name, x, y, yaw = best
        vx, vy, omega = truth.velocity(name)
        return target.Robot(
            position=spatial.Position[world](x, y, PLATE_Z),
            velocity=spatial.Vector[world](vx, vy, 0.0),
            acceleration=spatial.Vector[world](0.0, 0.0, 0.0),
            linear_uncertainty=spatial.LinearUncertainty[world](np.eye(3) * 1e-4),
            id=ENEMY_ID_BASE + i,
            icon=icons.IconType.SENTRY,
            theta=yaw,
            omega=omega,
            radius=PLATE_RADIUS,
            plates=[],
            plate_z_coords=[PLATE_Z] * 4,
        )

    original = turret_module.Turret.update_selector

    def update_selector(self: Any, force_reselection: bool) -> bool:
        robot = seen_enemy()
        had = getattr(self, "_hwemu_truth_target", None)
        if robot is None:
            if had is not None:  # lost it: no target, as when otto's tracker drops one
                self._hwemu_truth_target = None
                self.selected_scored_robot = None
                return True
            return original(self, force_reselection)  # otto's own (it has nothing to select)
        self._hwemu_truth_target = robot.id
        self.selected_scored_robot = selector.ScoredRobot(robot, SCORE)
        self.selected_filtered_robot = _TruthFilter(robot)
        return had != robot.id

    turret_module.Turret.update_selector = update_selector
