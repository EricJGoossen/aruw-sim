"""HWEMU stand-in for rm-pose-solver (Jetson-only wheel): see _yaw_solver.py."""
from ._yaw_solver import solve_yaw

__all__ = ["solve_yaw"]
