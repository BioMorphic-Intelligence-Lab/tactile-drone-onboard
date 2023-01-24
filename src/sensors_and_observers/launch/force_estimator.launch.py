import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('sensors_and_observers'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'sensors_and_observers',
        name = 'force_estimator_node',
        executable = 'force_estimator_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld