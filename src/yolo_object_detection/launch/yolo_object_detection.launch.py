from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Use a fixed path to your YAML file
    # Modify this path to point to your actual YAML file
    params_file = '/ros2_airsim/ws_ros2/src/yolo_object_detection/config/yolo_params.yaml'
    
    # Ensure the file exists and warn if it doesn't
    if not os.path.exists(params_file):
        print(f"WARNING: Parameters file not found at {params_file}")
    
    # Use the fixed path in the node configuration
    yolo_node = Node(
        package='yolo_object_detection',
        executable='yolo',
        name='yolo',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([yolo_node])