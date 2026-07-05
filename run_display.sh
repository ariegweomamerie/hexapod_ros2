#!/usr/bin/env bash
# Launch the hexapod RViz display OUTSIDE the VS Code snap sandbox.
# The VS Code snap injects GTK/GLib/locale env vars that point native GUI
# apps (RViz, joint_state_publisher_gui) at snap's incompatible libraries,
# causing "undefined symbol: __libc_pthread_init" crashes. We scrub them.

# Drop every snap-injected variable that poisons native GUI apps.
unset LD_LIBRARY_PATH
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR
unset GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH
unset GTK_MODULES

# Restore system XDG defaults (snap had rewritten these).
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
unset XDG_DATA_HOME
export XDG_CONFIG_DIRS=/etc/xdg

source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash

exec ros2 launch Hexapod_Robot_description display.launch.py "$@"
