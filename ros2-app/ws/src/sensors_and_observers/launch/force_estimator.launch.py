import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = "sensors_and_observers"

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'params.yaml'
        )
        
    # Start Wrench Transformer
    start_encoder_node = Node(package = pkg_name,
                              name = "encoder_node",
                              executable = "encoder_node",
                              parameters = [config])
    
    start_force_estimator_node=Node(
        package = pkg_name,
        name = 'force_estimator_node',
        executable = 'force_estimator_node',
        parameters = [config]
    )
    
    ld.add_action(start_encoder_node)
    ld.add_action(start_force_estimator_node)
    
    return ld