#!/usr/bin/env bash
# Point the Gazebo camera at the robot and keep it there.
#
#   ./follow_robot.sh            chase view, 1.6 m behind and 0.75 m up
#   ./follow_robot.sh 3 1.5      wider: 3 m behind, 1.5 m up
#   ./follow_robot.sh off        stop following, free camera again
#
# The offset is in the robot's own frame, where forward is -Y, so +Y is behind
# it. Gazebo forgets this whenever the simulator restarts, hence a script.
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
WORLD=hexapod_facility
if [ "$1" = "off" ]; then
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
    --timeout 4000 --req 'data: ""' && echo "camera released"
  exit 0
fi
BACK=${1:-1.6}
UP=${2:-0.75}
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 4000 --req 'data: "Hexapod_Robot"' >/dev/null
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
  --timeout 4000 --req "x: 0.0, y: ${BACK}, z: ${UP}" >/dev/null
echo "Gazebo camera following the robot: ${BACK} m behind, ${UP} m up"
