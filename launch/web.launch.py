import os
import sys
from pathlib import Path  # noqa: E402
from ament_index_python.packages import get_package_share_directory

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from camera_config import CameraConfig, USB_CAM_DIR  # noqa: E402

CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera1',
        param_path=Path(USB_CAM_DIR, 'config', 'params_1.yaml')
    )
    # Add more Camera's here and they will automatically be launched below
)


def generate_launch_description():
    home_dir = os.environ['HOME']
    
    vocabulary_path = os.path.join(home_dir, 'orb_ws/src/orb_slam3_ros_wrapper/config/ORBvoc.txt')
    settings_path = os.path.join(home_dir, 'orb_ws/src/orb_slam3_ros_wrapper/config/web_cam.yaml')
    
    camera_nodes = [
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
        for camera in CAMERAS
    ]

    camera_group = GroupAction(camera_nodes)

    rotate_node = Node(
        package='orb_slam3_ros_wrapper',
        executable='rotate.py',
        name='image_rotator',
        output='screen'
    )
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
            'publish_map_points': False,
            'publish_tracked_points': False,
            'use_viewer': False,
            'apply_world_frame_rotation': False
        }],
        remappings=[
            ('camera/image_raw', '/camera1/image_rotated')
        ]
    )
    
    return LaunchDescription([
        camera_group,
        rotate_node,
        static_transform_node,
        mono_node
    ])
