import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
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
        get_package_share_directory("omron1s_hwi"), "config", "controller_configuration.yaml"
    )

    world_file_name = 'empty_world.world'
    world = os.path.join(
        get_package_share_directory('omron1s_hwi'), 'worlds', world_file_name
    )

    return LaunchDescription([
        # Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[
        #         {"robot_description": robot_description_config.toxml()}, controller_config],
        #     output={
        #         "stdout": "screen",
        #         "stderr": "screen",
        #     },
        #     # prefix=["ethercat_grant"],
        #     # shell=True,
            
        # ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["joint_state_broadcaster",
        #                "-c", "/controller_manager"],
        #     parameters=[
        #         {"use_sim_time": True}],
        # ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["omron1s_manipulator_controller",
        #                "-c", "/controller_manager"],
        #     parameters=[
        #         {"use_sim_time": True}],
        # ),

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
                {"robot_description": robot_description_config.toxml()},
                # {"use_sim_time": True}
                ],
            output="screen"
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'omron1s', '-topic', 'robot_description'],
            output='screen',
            parameters=[
                # {"use_sim_time": True}
                ]
        ),
            

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], 
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','omron1s_manipulator_controller'],
            output='screen'
        ),

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
