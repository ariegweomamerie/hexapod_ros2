#!/usr/bin/env bash
# Launch the hexapod in Gazebo OUTSIDE the VS Code snap sandbox.
# Same snap-env scrub as run_display.sh (Gazebo's GUI hits the same crash).

unset LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
unset GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES

export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
unset XDG_DATA_HOME
export XDG_CONFIG_DIRS=/etc/xdg

# NOTE: if the gz GUI crashes with "Failed to create OpenGL context" (display
# only offers OpenGL < 3.3), either uncomment the software-GL fallback below, or
# run headless with:  ./run_gazebo.sh headless:=true   (visualize in RViz).
# export LIBGL_ALWAYS_SOFTWARE=1
# export GALLIUM_DRIVER=llvmpipe

source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash

exec ros2 launch Hexapod_Robot_description gazebo.launch.py "$@"
