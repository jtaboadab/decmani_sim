import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' robot_name:=wx250',
        ' base_link_frame:=base_link',
        ' use_world_frame:=false',
        ' external_urdf_loc:=""',
        ' use_gripper:=true'
    ]), value_type=str)
    
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
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
    )
    
    # Spawn del robot en Gazebo (posición definida por world_fixed joint en el URDF)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'wx250',
        ],
        output='screen'
    )

    # Bridge Gazebo → ROS2 para joint states y cámara
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/world/decmani_world/model/wx250/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        remappings=[
            ('/world/decmani_world/model/wx250/joint_state', '/joint_states'),
            ('/camera/image', '/camera/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ],
        output='screen'
    )

    # TF estático wx250/base_link → cámara
    # Calculado a partir de la pose SDF (pitch=0.7, yaw=pi):
    # gz rgbd_camera mira por +X del camera_link → camera_rgb +Z=forward, +X=right, +Y=down
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '1.465',
            '--y', '0.0',
            '--z', '0.6',
            '--qx', '-0.641071',
            '--qy', '-0.641071',
            '--qz', '0.298329',
            '--qw', '0.298329',
            '--frame-id', 'wx250/base_link',
            '--child-frame-id', 'camera/camera_link/camera_rgb'
        ],
        output='screen'
    )

    # Cargar y activar joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Cargar y activar arm_controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # Cargar y activar gripper_controller
    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )
    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        static_tf,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
    ])