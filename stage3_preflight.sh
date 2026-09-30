#!/usr/bin/env bash
# Stage 3 preflight: twenty read-only checks that must pass before any
# localisation experiment starts. Exit 0 = cleared, 1 = do not start.
#
#   ./stage3_preflight.sh                 full check, with the stack running
#   ./stage3_preflight.sh --offline       reference/config/process checks only
unset LD_LIBRARY_PATH GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES XDG_DATA_HOME
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
export XDG_CONFIG_DIRS=/etc/xdg
source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash
exec ros2 run hexapod_slam stage3_preflight -- "$@"
