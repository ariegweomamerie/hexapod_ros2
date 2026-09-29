#!/usr/bin/env bash
# Launch the hexapod inside the SLAM test facility, OUTSIDE the VS Code snap
# sandbox (same environment scrub as run_gazebo.sh).
unset LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
unset GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
unset XDG_DATA_HOME
export XDG_CONFIG_DIRS=/etc/xdg

source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash

# headless:=true for a server-only run (no GUI, higher real-time factor)
exec ros2 launch hexapod_worlds slam_world.launch.py "$@"
