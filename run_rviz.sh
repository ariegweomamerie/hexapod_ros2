#!/usr/bin/env bash
# Launch RViz2 ALONGSIDE Gazebo (snap-scrubbed, no extra robot_state_publisher /
# joint_state_publisher). Uses the running sim's /robot_description + /joint_states.

unset LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
unset GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
unset XDG_DATA_HOME
export XDG_CONFIG_DIRS=/etc/xdg

# Force Mesa software OpenGL (llvmpipe) - this display only exposes OpenGL 2.0,
# below what the 3D view needs, so hardware GL fails to create a context.
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash

RVIZ_CFG=/home/general/hexapod_ros_robot_ws/src/Hexapod_Robot_description/config/gazebo.rviz
exec rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
