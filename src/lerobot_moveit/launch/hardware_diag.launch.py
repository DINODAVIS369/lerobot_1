import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyACM0"
    )
    serial_port = LaunchConfiguration("serial_port")
    
    so_arm_100_hardware_dir = get_package_share_directory("so_arm_100_hardware")
    so101_xacro_path = os.path.join(so_arm_100_hardware_dir, "urdf", "so101.urdf.xacro")

    robot_description_content = Command([
        'xacro ', so101_xacro_path, ' serial_port:=', serial_port
    ])
    robot_description = {"robot_description": robot_description_content}

    ros2_control_config = PathJoinSubstitution([
        get_package_share_directory("lerobot_moveit"), "config", "ros2_control.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_control_config],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        control_node,
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=[gripper_spawner])),
    ])
