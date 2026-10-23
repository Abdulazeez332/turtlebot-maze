from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ai407_maze_project')
    world_file = os.path.join(pkg_share, 'worlds', 'phase3_7x7_maze.world')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    task_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ai407_maze_project',
                executable='maze_task_node',
                name='maze_task_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([gazebo, task_node])
