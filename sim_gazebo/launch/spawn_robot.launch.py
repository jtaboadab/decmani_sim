import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Paquetes
    pkg_sim_gazebo = get_package_share_directory('sim_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_xsarm_descriptions = get_package_share_directory('interbotix_xsarm_descriptions')
    
    # Variable de entorno para que Gazebo encuentre los meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_xsarm_descriptions)
    )

    # Archivos
    world_file = PathJoinSubstitution([pkg_sim_gazebo, 'worlds', 'decmani_world.sdf'])
    xacro_file = PathJoinSubstitution([pkg_sim_gazebo, 'urdf', 'wx250_gz.urdf.xacro'])
    
    # Procesar XACRO a URDF
    robot_description = Command([
        'xacro ', xacro_file,
        ' robot_name:=wx250',
        ' base_link_frame:=base_link',
        ' use_world_frame:=false',
        ' external_urdf_loc:=""',
        ' use_gripper:=true'
    ])
    
    # Nodo robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )
    
    # Spawn del robot en Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wx250',
            '-x', '-0.165',
            '-y', '0.0',
            '-z', '0.4'
        ],
        output='screen'
    )

    # Bridge Gazebo → ROS2 para la cámara
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        camera_bridge,
    ])