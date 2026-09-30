#!/usr/bin/env bash
# Bring up the complete Stage 2.4 SLAM stack - Gazebo, RViz, gait, RGB-D visual
# odometry and RTAB-Map - outside the VS Code snap sandbox, which otherwise
# crashes the GUI apps. One Ctrl-C stops all of it.
#
#   ./run_slam.sh                      full stack with both GUIs
#   ./run_slam.sh headless:=true       no Gazebo GUI, for benchmark runs
#   ./run_slam.sh slam:=false          Stage 2.3 stack only (no RTAB-Map)
unset LD_LIBRARY_PATH GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES XDG_DATA_HOME
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
export XDG_CONFIG_DIRS=/etc/xdg
source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash
exec ros2 launch hexapod_slam slam_stack.launch.py "$@"
