from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Launch Arguments
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for the robot hardware"
    )

    # Path to MoveIt and Hardware configs
    moveit_dir = get_package_share_directory('lerobot_moveit')
    hardware_dir = get_package_share_directory('so_arm_100_hardware')

    serial_port_value = LaunchConfiguration("serial_port")

    # ── MoveIt Configs ───────────────────────────────────────────────────────
    # Using absolute paths to ensure reliability across environments
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
        .robot_description(file_path="/home/dino_davis/lerobot_1/src/so_arm_100_hardware-main/urdf/so101.urdf.xacro")
        .robot_description_semantic(file_path="/home/dino_davis/lerobot_1/src/lerobot_moveit/config/so101.srdf")
        .robot_description_kinematics(file_path="/home/dino_davis/lerobot_1/src/lerobot_moveit/config/kinematics.yaml")
        .to_moveit_configs()
    )

    # ── Servo Config ──────────────────────────────────────────────────────────
    teleop_dir = get_package_share_directory('so101_servo_teleop')
    servo_config = os.path.join(teleop_dir, 'config', 'servo.yaml')

    with open(servo_config, 'r') as f:
        import yaml
        servo_params = yaml.safe_load(f)

    # ── Joint-state relay ─────────────────────────────────────────────────────
    joint_state_relay_node = Node(
        package="so101_servo_teleop",
        executable="joint_state_relay",
        output="screen",
    )

    # ── MoveIt Servo node ────────────────────────────────────────────────────
    # PlanningSceneMonitor options for Jazzy compatibility
    psm_params = {
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states_clean',
        }
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            {'moveit_servo': servo_params},
            psm_params,
        ],
        output="screen"
    )

    return LaunchDescription([
        serial_port_arg,
        joint_state_relay_node,
        TimerAction(period=3.0, actions=[servo_node]),
    ])

