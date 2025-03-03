
import launch 
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import sys
import subprocess

# Launch file for misao process from
# kurome. Mostly just provides the loading
# of parameters from a config file

def generate_launch_description():

    # get config file
    config = os.path.join(
            get_package_share_directory('kurome'),
            'config',
            'misao.yaml'
    )

    # the node
    misao_node = launch_ros.actions.Node(
            package='kurome',
            executable='misao',
            namespace='kurome',
            name='misao',
            parameters=[config]
    )

    return launch.LaunchDescription([
        misao_node
    ])
