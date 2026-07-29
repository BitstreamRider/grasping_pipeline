from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = '/root/ros2_ws/TUW_maps/map.yaml'

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',            
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    return LaunchDescription([map_server, amcl, lifecycle_manager])