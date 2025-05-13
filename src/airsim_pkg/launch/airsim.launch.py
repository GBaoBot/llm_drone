from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create image publisher node with directly specified parameters
    image_publisher_node = Node(
        package='airsim_pkg',
        executable='airsim_image_publisher',
        name='airsim_image_publisher',
        parameters=[{
            'image_topic': 'airsim/image',
            'camera_id': '0',
            'frequency': 10.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_publisher_node
    ])
