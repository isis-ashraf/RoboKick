import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf', 'robot.urdf'
    )

    # Read the URDF file contents
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # robot_state_publisher: reads the URDF and publishes
        # TF transforms so RViz can draw the robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui: gives you sliders in a
        # window to manually move each joint in RViz
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz: the 3D visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),

    ])