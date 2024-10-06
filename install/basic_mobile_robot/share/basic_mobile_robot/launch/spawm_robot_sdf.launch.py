import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    # General arguments
    robot_type = LaunchConfiguration("robot_type").perform(context)

    # RViz config file path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("basic_mobile_robot"), "rviz", "visualize_robot.rviz"]
    )

    # Path to the model.sdf file
    sdf_file_path = PathJoinSubstitution(
        [FindPackageShare("basic_mobile_robot"), "models", "robotV1_description", "model.sdf"]
    )

    # Robot state publisher for publishing robot model
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}],
    )

    # Gazebo node with empty world
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
    )

    # Node to spawn robot model in Gazebo from SDF file
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sdf_file_path, '-entity', 'robot'],
        output='screen',
    )

    # RViz node for visualizing robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay start of RViz until the robot is spawned in Gazebo
    delay_rviz_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[
                LogInfo(msg='Spawned robot model in Gazebo. Starting RViz'),
                rviz_node
            ],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        gazebo_node,
        spawn_entity_node,
        delay_rviz_after_spawn_entity,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_type",
            default_value="omni",
            description="Type of robot to visualize (Available options: omni, diff)",
        )
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
