#!/usr/bin/env bash
# Bring up the complete Stage 3 localisation stack - Gazebo, RViz, gait, RGB-D
# visual odometry and RTAB-Map localising against the FROZEN Stage 2.4 reference
# map - outside the VS Code snap sandbox, which otherwise crashes the GUI apps.
#
#   ./run_localization.sh run_dir:=verification/runs/stage3_L0_$(date +%Y%m%d_%H%M%S)
#   ./run_localization.sh run_dir:=... headless:=true      # no Gazebo GUI
#   ./run_localization.sh run_dir:=... localization:=false # sensing + odometry only
#
# The launch verifies the reference database (sha256, SQLite integrity, visual
# vocabulary) and copies it to run_dir/reference.db before RTAB-Map starts. The
# canonical reference is never opened by an experiment.
#
# Run ./stage3_preflight.sh first - it will tell you not to start if anything
# about the reference, the configuration or the running processes is wrong.
unset LD_LIBRARY_PATH GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
unset LOCPATH GTK_MODULES XDG_DATA_HOME
export XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
export XDG_CONFIG_DIRS=/etc/xdg
source /opt/ros/jazzy/setup.bash
source /home/general/hexapod_ros_robot_ws/install/setup.bash

# Refuse to bring up a SECOND stack. Two Gazebo servers, two odometry nodes or
# two RTAB-Maps on the same topics produce data that looks plausible and is
# meaningless; this project has already lost a run that way. Override with
# STAGE3_FORCE=1 only if you know the existing processes are unrelated.
#
# Self-exclusion matters: this script's own command line contains the patterns
# being searched for, so a naive `pgrep -f` matches the guard itself and the
# guard then refuses every time.
if [ -z "$STAGE3_FORCE" ]; then
  running=$(ps -eo pid,args --no-headers | awk -v me="$$" -v pp="$PPID" '
    $1 != me && $1 != pp && $0 !~ /run_localization\.sh/ && $0 !~ /awk/ {
      if (index($0, "gz sim server"))                 c["gz sim server"]++
      if (index($0, "rtabmap_odom/rgbd_odometry"))    c["rgbd_odometry"]++
      if (index($0, "rtabmap_slam/rtabmap"))          c["rtabmap"]++
      if (index($0, "hexapod_gait/gait_node"))        c["gait_node"]++
      if (index($0, "ros_gz_bridge/parameter_bridge")) c["parameter_bridge"]++
      if (index($0, "drive_facility_loop"))           c["waypoint follower"]++
      if (index($0, "bag record"))                    c["rosbag recorder"]++
    }
    END { for (k in c) printf "  %s x%d\n", k, c[k] }')
  if [ -n "$running" ]; then
    echo "refusing to start: part of the stack is already running:"
    echo "$running"
    echo "stop it first, or re-run with STAGE3_FORCE=1 if this is intentional."
    exit 1
  fi
fi

exec ros2 launch hexapod_slam localization_stack.launch.py "$@"
