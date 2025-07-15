"""
A launch file for running the motion planning python api tutorial
"""
import os
import sys
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = "true"
    log_level = 'error'

    for arg in sys.argv:
        if arg.startswith("info:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'info'
        if arg.startswith("debug:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'debug'
    
    print("")
    print("log_level:           " + str(log_level))
    print("")

    # Define xacro mappings for the robot description file
    launch_arguments = {
            "use_fake_hardware": use_fake_hardware,
            }
    moveit_config = (
            MoveItConfigsBuilder(
                robot_name="jonny", package_name="jonny_moveit_config"
                )
            .robot_description(mappings=launch_arguments)
            .to_moveit_configs()
            )

    ld = LaunchDescription()

    #################### Rviz controlled moveit node
    move_group_configuration = {
        "publish_robot_description_semantic": True,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
    move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
            arguments=['--ros-args', '--log-level', log_level]
            )
    ld.add_action(move_group_node)

    # rviz control
    rviz_config_file = os.path.join(
            get_package_share_directory("jonny_moveit_config"),
            "config",
            "moveit.rviz",
            )
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file, '--ros-args', '--log-level', log_level],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                ],
            )

    ld.add_action(rviz_node)

    #################### Moveit Components
    static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["--frame-id", "world", "--child-frame-id", "base_link", '--ros-args', '--log-level', log_level],
            )
    ld.add_action(static_tf)

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="log",
            parameters=[moveit_config.robot_description],
            arguments=['--ros-args', '--log-level', log_level],
            )
    ld.add_action(robot_state_publisher)

    ros2_controllers_path = os.path.join(
            get_package_share_directory("jonny_moveit_config"),
            "config",
            "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    ld.add_action(ros2_control_node)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(str(moveit_config.package_path) + "/launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
