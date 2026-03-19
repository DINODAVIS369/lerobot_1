
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for the arm",
        )
    )

    serial_port = LaunchConfiguration("serial_port")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("so_arm_100_hardware"), "urdf", "so101.urdf.xacro"]
            ),
            " ",
            "serial_port:=",
            serial_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ---------------- Controllers YAML ----------------
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("so_arm_100_hardware"), "config", "ros2_control.yaml"]
    )

    # ---------------- Robot State Publisher ----------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # ---------------- ros2_control ----------------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file
        ],
        output="screen",
    )

    # ... rest of the spawners ...
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_controller_spawning = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        control_node,
        delayed_controller_spawning,
    ])
