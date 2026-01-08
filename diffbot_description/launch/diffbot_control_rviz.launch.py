#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("diffbot_description")

    # -------- Paths --------
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "diffbot.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    default_rviz_config = PathJoinSubstitution([pkg_share, "rviz", "odom_tf_view.rviz"])

    # -------- Arguments --------
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # -------- robot_description (xacro -> robot_description) --------
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str
        )
    }

    # -------- ros2_control controller_manager --------
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -------- joint_state_broadcaster spawner --------
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # -------- imu_sensor_broadcaster spawner --------
    imu_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    

    # -------- diff_drive_controller spawner --------
    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # -------- robot_state_publisher (RobotModel/TF용) --------
    # joint_state_broadcaster가 /joint_states를 publish -> robot_state_publisher가 TF를 생성
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # -------- RViz2 --------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Gazebo/시뮬이면 true, 실제 하드웨어면 false",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file path",
        ),

        control_node,
        jsb_spawner,
        imu_spawner,
        diff_spawner,
        rsp_node,
        rviz_node,
    ])
