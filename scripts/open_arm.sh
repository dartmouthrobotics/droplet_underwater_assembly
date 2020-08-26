#! /bin/bash
# arm the robot
echo "Starting gripper open sequence"
echo "Arming robot"
rosservice call /mavros/cmd/arming "value: true"

echo "Moving gripper"
rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1560, 1500, 1500, 1500]" --once
sleep 2
rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1560, 1500, 1500, 1500]" --once
sleep 1

echo "Stopping"
rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]" --once

echo "Disarming robot"
rosservice call /mavros/cmd/arming "value: false"
