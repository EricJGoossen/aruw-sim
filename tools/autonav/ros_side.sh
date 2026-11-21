#!/usr/bin/env bash
# The ROS 2 side of a scripts/run.sh --autonav run, in the autonav container:
# the stand-in Odin driver (hwemu_odin.py: the Odin's cloud, its pose with its
# measured error (assets/sentry/odin.yaml), and
# /clock from the hub) and the branch's localization launch (robot_state_
# publisher, map_server, AMCL, the localization node), on sim time, against
# <run dir>/autonav/field.yaml with <run dir>/autonav/localization.yaml.
#
#   ros_side.sh RUN_DIR AUTONAV_INSTALL
set -euo pipefail
RT="$1" WS="$2"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "$WS/setup.bash"
set -u
trap 'kill 0 2>/dev/null' EXIT
python3 -u "$here/hwemu_odin.py" "$RT" "$here/../../assets/sentry/odin.yaml" &
ros2 launch localization localization.launch.py \
  map:="$RT/autonav/field.yaml" node_params:="$RT/autonav/localization.yaml" use_sim_time:=true &
wait -n
