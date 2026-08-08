import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'hexapod_description_final'
    pkg_share = get_package_share_directory(pkg_name)

    # Chemin du fichier URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'hexapod_description_final.urdf')

    # Si le fichier URDF n'a pas _final dans son nom, tente sans _final
    if not os.path.exists(urdf_file):
        urdf_file = os.path.join(pkg_share, 'urdf', 'hexapod_description.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Noeud Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Noeud RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])