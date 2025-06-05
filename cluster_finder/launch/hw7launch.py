# The launch file for HW7 for OSU ROB499 Robotic Software Frameworks
#
# hw7launch.py
#
# Nathan Martin

import launch
import launch_ros.actions
import os.path as path
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return launch.LaunchDescription([

        # Launch for python node cluster finder
        launch_ros.actions.Node(
            package='cluster_finder',
            executable='cluster_finder',          
        ),

        # Launch for C++ node clean_space
        launch_ros.actions.Node(
            package='clean_space',
            executable='clean_space',         
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', path.join(get_package_share_directory('cluster_finder'), 'config', 'rvizconfig.rviz')],
            name='rviz2',
            output='screen'  
        ),
        ])
