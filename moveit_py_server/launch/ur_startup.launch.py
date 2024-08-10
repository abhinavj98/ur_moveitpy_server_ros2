#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode", default="false")

    initial_joint_controller = LaunchConfiguration(
        "initial_joint_controller", default="scaled_joint_trajectory_controller"
    )
    set_joint_controller = SetLaunchConfiguration(
        "initial_joint_controller",
        value="joint_trajectory_controller",
        condition=LaunchConfigurationEquals("use_fake_hardware", "true"),
    )

    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5e", description="Robot description name (consistent with ur_control.launch.py)"
    )
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="169.254.174.50", description="Robot IP")

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="false", description="If true, uses the fake controllers"
    )

    ur_base_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_robot_driver"), "launch/ur_control.launch.py")
        ),
        launch_arguments=[
            ("robot_ip", robot_ip),
            ("ur_type", ur_type),
            ("use_fake_hardware", use_fake_hardware),
            ("headless_mode", headless_mode),
            ("initial_joint_controller", initial_joint_controller),
            ("launch_rviz", "false"),
        ],
    )

    ur_moveitpy_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory("moveit_py_server"), "launch/ur_moveitpy.launch.py")
        ),
        launch_arguments=[
            ("ur_type", ur_type),
            ("use_fake_hardware", use_fake_hardware),
            ("launch_rviz", "true"),
        ],
    )
    
    joy_node = Node(
            package="joy",
            executable="joy_node",
        )
    
    

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_fake_hardware_arg,
            set_joint_controller,
            ur_base_launch,
            ur_moveitpy_launch,
            joy_node,
        ]
    )