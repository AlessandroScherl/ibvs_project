import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('ibvs_simulation')
    world_file = os.path.join(pkg_share, 'worlds', 'ibvs_world.sdf')
    
    # Launch Gazebo (new gz-sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Bridge for ROS2 <-> Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ibvs/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/ibvs/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/ibvs/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/ibvs/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rgbd_camera_robot',
            '-file', os.path.join(pkg_share, 'models', 'rgbd_camera_robot', 'model.sdf'),
            '-x', '0',
            '-y', '0',
            '-z', '1'
        ],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config = os.path.join(pkg_share, 'config', 'ibvs.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        bridge,
        spawn_robot,
        rviz
    ])
