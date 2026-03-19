#!/bin/bash
export ROS_DOMAIN_ID=10
source install/setup.bash
echo "Starting Keyboard Teleop (Domain 10)..."
# Background loop to unpause servo once it starts
(
    for i in {1..30}; do
        if ros2 service list | grep -q "/servo_node/pause_servo"; then
            sleep 2
            ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}"
            break
        fi
        sleep 2
    done
) &

ros2 launch so101_servo_teleop servo.launch.py
