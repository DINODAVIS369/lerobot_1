import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Launch Arguments
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for the robot hardware"
    )

    serial_port = LaunchConfiguration("serial_port")

    # URDF path
    so_arm_100_hardware_dir = get_package_share_directory("so_arm_100_hardware")
    so101_xacro_path = os.path.join(so_arm_100_hardware_dir, "urdf", "so101.urdf.xacro")

    # Load URDF with serial_port parameter override using xacro
    robot_description_content = Command(
        [
            'xacro ',
            so101_xacro_path,
            ' serial_port:=',
            serial_port
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
            MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
            .robot_description(file_path=so101_xacro_path, mappings={"serial_port": LaunchConfiguration("serial_port")})
            .robot_description_semantic(file_path="config/so101.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .robot_description_kinematics()
            .joint_limits()
            .to_moveit_configs()
            )

    # Get path to ros2_control.yaml
    ros2_control_config = PathJoinSubstitution(
        [get_package_share_directory("lerobot_moveit"), "config", "ros2_control.yaml"]
    )

    # ros2_control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            ros2_control_config
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner for arm_controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Spawner for gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # moveit core
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), robot_description, {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViZ
    rviz_config_path = os.path.join(get_package_share_directory("lerobot_moveit"),"config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
                    robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    # Static Transform (if needed, but URDF should have it)
    
    # --- SEQUENCING ---

    # 1. Spawn joint_state_broadcaster after control_node starts
    spawn_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # 2. Spawn arm_controller after joint_state_broadcaster finishes
    spawn_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # 3. Spawn gripper_controller after arm_controller finishes
    spawn_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # 4. Enable Torque automatically after gripper_controller finishes
    # enable_torque = ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/toggle_torque', 'std_srvs/srv/Trigger', '{}'],
    #     output='screen'
    # )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        control_node,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
        move_group_node,
        rviz_node
    ])
