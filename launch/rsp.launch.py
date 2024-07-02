import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    print("Starting to generate launch description...")

    use_sim_time = LaunchConfiguration('use_sim_time')
    print(f"use_sim_time: {use_sim_time}")

    pkg_path = os.path.join(get_package_share_directory('indoorlogistics'))
    print(f"Package path: {pkg_path}")

    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')  # Update path here
    print(f"Xacro file: {xacro_file}")

    # Ensure the xacro file is valid
    try:
        robot_description_config = xacro.process_file(xacro_file)
        robot_description = robot_description_config.toxml()
        print(f"Robot description: {robot_description[:100]}...")  # Print the first 100 characters
    except Exception as e:
        print(f"Failed to process xacro file: {e}")
        return LaunchDescription([])

    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    print("Node created")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher
    ])

if __name__ == '__main__':
    generate_launch_description()

