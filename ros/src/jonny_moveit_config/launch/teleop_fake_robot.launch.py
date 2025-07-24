"""
A launch file for running the motion planning python api tutorial
"""
import os
import sys
import json
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import ParameterBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = "true"
    log_level = 'error'
    rviz = "false"

    for arg in sys.argv:
        if arg.startswith("info:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'info'
        if arg.startswith("debug:="):
            content = arg.split(":=")[1]
            if content == "true":
                log_level = 'debug'
        if arg.startswith("rviz:="):
            content = arg.split(":=")[1]
            if content == "true":
                rviz = "true"
    
    print("")
    print("log_level:           " + str(log_level))
    print("rviz:                " + str(rviz))
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

    #################### Move Group
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
            output="log",
            parameters=move_group_params,
            arguments=['--ros-args', '--log-level', "fatal"]
            )
    ld.add_action(move_group_node)

    # #################### Rviz
    if (rviz == "true"):
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

    #################### Servo
    servo_params = {
        "moveit_servo": ParameterBuilder("jonny_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm"}

    servo_node = launch_ros.actions.Node(
        package="jonny_servo",
        executable="moveit_servo_interface",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )
    ld.add_action(servo_node)

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

def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        try:
            with open(absolute_file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
            return None
