#!/usr/bin/env bash
# Run the harness on Isaac's bundled ROS 2 Jazzy (Python 3.11), not the system one (3.12),
# so rclpy imports. System ROS nodes in other terminals still connect over DDS.
B="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy"
exec env -i HOME="$HOME" USER="$USER" DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" PATH=/usr/bin:/bin \
    ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp LD_LIBRARY_PATH="$B/lib" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$HOME/isaacsim/python.sh" "$(dirname "$(realpath "$0")")/harness.py" "$@"
