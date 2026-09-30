#!/usr/bin/env bash
# Open the Stage 2.4 SLAM view on its own, OUTSIDE the VS Code snap sandbox.
# The facility launcher already brings RViz up with this same config; use this
# when you want to reopen or reload the view without restarting the simulation.
unset LD_LIBRARY_PATH GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES XDG_DATA_HOME
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
export XDG_CONFIG_DIRS=/etc/xdg
source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash
exec ros2 run rviz2 rviz2 -d \
  "$(ros2 pkg prefix hexapod_slam)/share/hexapod_slam/rviz/stage2_4_slam.rviz" \
  --ros-args -p use_sim_time:=true
