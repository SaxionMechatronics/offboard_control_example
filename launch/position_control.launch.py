import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('offboard_control'),
        'config',
        'position_control.yaml'
        )
        
    node=Node(
        package = 'offboard_control',
        name = 'position_control',
        executable = 'position_control',
        parameters = [config],
        emulate_tty=True,
    )
    ld.add_action(node)
    return ld