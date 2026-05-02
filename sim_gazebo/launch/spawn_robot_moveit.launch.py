import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paquetes
    pkg_sim_gazebo = get_package_share_directory('sim_gazebo')
    pkg_moveit = get_package_share_directory('wx250_moveit_config')
    
    # Forzar use_sim_time en todos los nodos
    use_sim_time = SetParameter(name='use_sim_time', value=True)

    # Tu launch actual de Gazebo + robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sim_gazebo, 'launch', 'spawn_robot.launch.py'])
        ])
    )
    
    # Solo el move_group de MoveIt
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_moveit, 'launch', 'move_group.launch.py'])
        ])
    )
    
    # RViz con configuración de MoveIt
    rviz_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_moveit, 'launch', 'moveit_rviz.launch.py'])
        ])
    )

    return LaunchDescription([  
        use_sim_time,
        gazebo_launch,
        move_group,
        rviz_moveit,
    ])