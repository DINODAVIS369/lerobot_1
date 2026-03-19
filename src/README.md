# SO-ARM-100 MoveIt 2 & Hardware Workspace

This workspace contains the software stack to control the **SO-ARM-100** robot arm using ROS 2 Jazzy, MoveIt 2, and `ros2_control`.

## 📂 Project Structure

*   `src/so_arm_100_hardware-main`: Hardware interface plugin that communicates with the ST3215 servos over Serial (RS485).
*   `src/lerobot_moveit`: MoveIt 2 configuration, launch files, and trajectory controllers.
*   `src/lerobot_moveit/scripts`: Python testing scripts to verify hardware motion.

## 🛠️ Installation & Build

### Prerequisites
*   ROS 2 Jazzy (installed on Ubuntu 24.04).
*   SCServo SDK and dependencies (included in hardware package).

### Building
```bash
# Navigate to workspace root
cd ~/lerobot_1

# Build the packages
colcon build --packages-select so_arm_100_hardware lerobot_moveit

# Source the workspace
source install/setup.bash
```

## 🚀 How to Run

### 1. Launch MoveIt & Hardware
Connect the robot arm USB to your computer and ensure it is powered on. Then run:
```bash
ros2 launch lerobot_moveit so101_moveit.launch.py
```
This will:
*   Load the URDF/SRDF.
*   Start `robot_state_publisher` (TF tree).
*   Initialize the `ros2_control_node` (Serial connection to servos).
*   Spawn `arm_controller`, `gripper_controller`, and `joint_state_broadcaster`.
*   Launch RViZ for visualization.

### 2. Verify Controllers
In a new terminal:
```bash
source install/setup.bash
ros2 control list_controllers
```
Ensure all three controllers are marked as **active**.

### 3. Move the Robot
To test motion on the real hardware, run the provided script:
```bash
python3 src/lerobot_moveit/scripts/move_arm.py
```

## 🔧 Hardware Settings
*   **Serial Port**: Currently configured as `/dev/ttyACM0` in `so101.urdf`.
*   **Baud Rate**: `1000000`.
*   **Torque**: If the arm is "limp", call the torque toggle service:
    ```bash
    ros2 service call /toggle_torque std_srvs/srv/Trigger "{}"
    ```

## ⚠️ Troubleshooting
*   **"No root/virtual joint specified"**: Common warning in MoveIt, ignored for fixed-base arms.
*   **"Overrun detected"**: The hardware read loop is slow (~80ms). This is expected for high-current serial polling but handled by generous tolerances in `ros2_control.yaml`.
*   **Serial Permissions**: Ensure your user is in the `dialout` group: `sudo usermod -a -G dialout $USER` (requires logout/login).
