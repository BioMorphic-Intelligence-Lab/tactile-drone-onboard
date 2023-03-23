import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = "planners"

    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'params.yaml'
        )
        
    # Start Wrench Transformer
    start_base_reference_publisher = Node(package = pkg_name,
                                          name = "base_reference_position_publisher",
                                          executable = "base_reference_position_publisher",
                                          parameters = [config])
    
    start_contact_based_planner = Node(package = pkg_name,
                                       name = 'contact_based_reference_finder',
                                       executable = 'contact_based_reference_finder',
                                       parameters = [config]
    )
    
    ld.add_action(start_base_reference_publisher)
    ld.add_action(start_contact_based_planner)
    
    return ld