from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_robot_py',
            executable='arm_robot',
            name='arm_robot_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])
