# Sentry Mk2 Coarse URDF Package (`sentry_description`)

## Quick Start — 3D Interactive Web Visualizer (No ROS Required)
```bash
python scripts/run_visualizer.py --port 8080
```
Opens `http://127.0.0.1:8080/visualizer/index.html` with 7-DOF joint sliders, visual/collision mesh toggles, coordinate frames, joint axis arrows, and COG markers.

## URDF Models (`urdf/`)
- `urdf/sentry_coarse.urdf`: Production URDF using `package://sentry_description/meshes/...` URIs (for ROS 1 / ROS 2 / RViz).
- `urdf/sentry_coarse_standalone.urdf`: Standalone URDF using relative `meshes/...` paths (for direct loading in MuJoCo, Isaac Sim, PyBullet, Pinocchio, Drake, or Blender without ROS).
- `urdf/sentry_primitives.urdf`: Pure analytical box/cylinder URDF with zero mesh dependencies.

## Meshes (`urdf/meshes/`)
- `urdf/meshes/visual/*.stl`: Clean, uncorrupted SolidWorks CAD visual meshes (`chassis.stl`, `omni_wheel.stl`, `turret_major.stl`, `turret_minor_frame.stl`, `turret_minor_firing_system.stl`).
- `urdf/meshes/collision/*.stl`: Watertight convex-hull collision meshes (`0.40 MB` total across all 5 links).

## Kinematic & Physical Summary
- **Total Mass**: `19.6313 kg` across 8 rigid links + `base_footprint`
- **Actuated Joints (7 DOF)**:
  1. `joint_fl_wheel` (`continuous`, axis `[0.707107, 0.707107, 0]`)
  2. `joint_fr_wheel` (`continuous`, axis `[0.707107, -0.707107, 0]`)
  3. `joint_bl_wheel` (`continuous`, axis `[-0.707107, 0.707107, 0]`)
  4. `joint_br_wheel` (`continuous`, axis `[-0.707107, -0.707107, 0]`)
  5. `joint_major` (`continuous`, yaw around major bearing at `Z = 0.0776 m`, axis `[0, 0, 1]`)
  6. `joint_minor_yaw` (`revolute`, yaw around GM6020 at base of minor frame `Z = 0.13935 m`, axis `[0, 0, 1]`, `[-90°, +90°]`)
  7. `joint_minor_pitch` (`revolute`, pitch around shoulder screw on firing system / neck insert `Z = 0.214 m`, axis `[0, 1, 0]`, `[-20°, +35°]`)
