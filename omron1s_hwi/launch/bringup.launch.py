import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    robot_name = "omron1s"
    robot_model_file = "omron1s.xacro"
    package_name = "omron1s_description"

    rviz_config = os.path.join(get_package_share_directory(
        package_name), "rviz", "omron1s.rviz")

    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", robot_model_file)

    robot_description_config = xacro.process_file(robot_description)

    controller_config = os.path.join(
        get_package_share_directory(
            "omron1s_hwi"), "config", "controller_configuration.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            # prefix=["ethercat_grant"],
            # shell=True,
            
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["omron1s_manipulator_controller",
                       "-c", "/controller_manager"],
        ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["forward_position_controller",
        #                "-c", "/controller_manager"],
        # ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     arguments=["-d", rviz_config],
        #     output={
        #         "stdout": "screen",
        #         "stderr": "log",
        #     },
        # )

    ])
