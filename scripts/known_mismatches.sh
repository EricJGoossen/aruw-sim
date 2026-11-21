# The model mismatches tools/check_model.py reports today, which you (the
# owner) are to resolve. Sourced by smoke.sh for test runs
# so they don't stop the build. They are NOT fixed, only let through, and
# check_model.py still prints each one. For a run of your own:
#   source scripts/known_mismatches.sh && scripts/run.sh
#
# - wheelbase_radius.joint_*_wheel: URDF 0.2216 m vs MCB WHEELBASE_RADIUS 0.23864 m
# - joint_minor_yaw.limits:  URDF +-90 deg vs firmware +-143.5 deg
# - joint_minor_pitch.limits: URDF -20..35 deg vs firmware -13..40 deg
export HWEMU_ACCEPT_MISMATCH="wheelbase_radius.joint_fl_wheel,wheelbase_radius.joint_fr_wheel,wheelbase_radius.joint_bl_wheel,wheelbase_radius.joint_br_wheel,joint_minor_yaw.limits,joint_minor_pitch.limits"
