from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('px4_ros2_control')
    config_file = os.path.join(pkg_dir, 'config', 'helicopter.yaml')

    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")

    control_node = Node(
        package='px4_ros2_control',
        executable='control_mode',
        name='control_mode_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        control_node
    ])