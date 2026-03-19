#!/bin/bash
export ROS_DOMAIN_ID=10

echo "------------------------------------------------"
echo "ROBOT STARTUP & RECOVERY SCRIPT (Idempotent)"
echo "------------------------------------------------"

# 1. Kill old processes with absolute prejudice
echo "Cleaning up old ROS 2 processes..."
pkill -9 -f ros2 || true
pkill -9 -f "_node" || true
sleep 2

# 1.5 Standalone Hardware Check
echo "Checking Serial Health..."
MAX_RETRIES=5
for i in $(seq 1 $MAX_RETRIES); do
    if [ -e "/dev/ttyACM0" ]; then
        python3 check_hardware.py /dev/ttyACM0
        if [ $? -eq 0 ]; then
            echo "Hardware is HEALTHY."
            break
        fi
    fi
    echo "Waiting for robot to connect (Retry $i/$MAX_RETRIES)..."
    sleep 3
    if [ $i -eq $MAX_RETRIES ]; then
        echo "ERROR: Robot not detected after $MAX_RETRIES retries."
        echo "Please check the 12V power and USB cable!"
        exit 1
    fi
done

# 2. Check Serial Port
SERIAL_PORT=${1:-/dev/ttyACM0}
if [ ! -e "$SERIAL_PORT" ]; then
    echo "ERROR: Serial port $SERIAL_PORT not found!"
    echo "Please check the USB cable and robot power."
    exit 1
fi
echo "Serial port detected: $SERIAL_PORT"

# 3. Rebuild and Source
echo "Rebuilding packages..."
colcon build --symlink-install --packages-select so101_servo_teleop lerobot_moveit
source install/setup.bash

# 4. Launch Stack
echo "Starting MoveIt and Hardware..."
ros2 launch lerobot_moveit so101_moveit.launch.py serial_port:=$SERIAL_PORT --noninteractive &
LAUNCH_PID=$!

# 5. Smart Controller Activation
echo "Waiting for Controller Manager..."
for i in {1..30}; do
    if ros2 service list | grep -q "/controller_manager/list_controllers"; then
        echo "Controller Manager is UP!"
        break
    fi
    sleep 2
done

activate_if_needed() {
    local name=$1
    local status=$(ros2 control list_controllers | grep "^$name" | awk '{print $2}')
    
    if [ "$status" == "active" ]; then
        echo "$name is already active."
        return
    fi

    echo "Activating $name (Current status: ${status:-not loaded})..."
    if [ -z "$status" ]; then
        ros2 service call /controller_manager/load_controller controller_manager_msgs/srv/LoadController "{name: '$name'}"
    fi
    ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: '$name'}"
    ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['$name'], deactivate_controllers: [], strictness: 1}"
}

sleep 2
activate_if_needed "joint_state_broadcaster"
activate_if_needed "arm_controller"
activate_if_needed "gripper_controller"

# 6. Wait for MoveGroup
echo "Waiting for MoveGroup..."
for i in {1..15}; do
    if ros2 node list | grep -q "/move_group"; then
        echo "MoveGroup is UP!"
        break
    fi
    sleep 2
done

echo "------------------------------------------------"
echo "System is Ready!"
echo "Keep THIS Terminal open!"
echo ""
echo "Now run teleop in a NEW terminal with:"
echo "  cd ~/lerobot_1/src"
echo "  ./fix_teleop.sh"
echo "------------------------------------------------"

# Keep script running
wait $LAUNCH_PID
