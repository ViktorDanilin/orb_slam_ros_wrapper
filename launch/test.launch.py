import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    home_dir = os.environ['HOME']
    
    vocabulary_path = os.path.join(home_dir, 'visual_odometry/orb_ws/src/orb_slam3_ros_wrapper/config/ORBvoc.txt')
    settings_path = os.path.join(home_dir, 'visual_odometry/orb_ws/src/orb_slam3_ros_wrapper/config/web_cam.yaml')
    
    path_to_rosbag_arg = DeclareLaunchArgument(
        'pathToRosbag',
        default_value='/home/viktor/Downloads/rosbag2_2025_05_13-13_12_53/',
        description='Path to the ROS bag file'
    )

    bag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('pathToRosbag'),
            '--topics', '/camera/image_raw'],
        name='bag_player',
        output='screen'
    )

    # # Нода для поворота изображения
    # rotate_node = Node(
    #     package='orb_slam3_ros_wrapper',
    #     executable='rotate.py',
    #     name='image_rotator',
    #     output='screen'
    # )
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'orb_slam3_map'],
        name='static_transform_publisher'
    )

    mono_node = Node(
        package='orb_slam3_ros_wrapper',
        executable='mono_node',
        name='orb_slam3_mono',
        output='screen',
        parameters=[{
            'vocabulary_file': vocabulary_path,
            'settings_file': settings_path,
            'world_frame_id': 'map',
            'camera_frame_id': 'camera_link',
            'publish_tf': True,
            'publish_pose': True,
            'publish_orb_pose': True,
            'publish_map_points': True,
            'publish_tracked_points': True,
            'use_viewer': True,
            'apply_world_frame_rotation': False
        }],
        # remappings=[
        #     ('camera/image_raw', '/cam_ros/image_rotated')
        # ]
    )
    
    return LaunchDescription([
        path_to_rosbag_arg,
        # rotate_node,
        static_transform_node,
        mono_node,
        bag_play_node
    ])