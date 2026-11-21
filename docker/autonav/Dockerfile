# ROS 2 Humble for the 2026-27 autonav stack (project-otto
# feature/autonav_framework: autonav/ and project_otto/bridge), next to the
# hub and the MCB, which stay on the host: otto runs here, since its bridge
# imports rclpy, and Humble's rclpy is CPython 3.10 like otto's venv.
# scripts/autonav.sh builds and runs it.
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-msgs ros-humble-robot-state-publisher ros-humble-xacro \
        ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-sensor-msgs-py \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions python3-numpy python3-yaml python3-serial \
        libgl1 libglib2.0-0 libegl1 procps psmisc iproute2 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.9.7 /uv /uvx /usr/local/bin/
