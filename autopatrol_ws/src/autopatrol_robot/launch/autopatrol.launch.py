import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    default_patrol_config = os.path.join(get_package_share_directory('autopatrol_robot'), 'config', 'patrol_config.yaml')
    # Get package paths
    robot_description_dir = get_package_share_directory('robot_description')
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    
    # Include Gazebo simulation launch file
    gazebo_sim_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_description_dir, '/launch', '/gazebo_sim.launch.py']
        )
    )
    
    # Include Navigation2 launch file
    navigation2_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_navigation2_dir, '/launch', '/navigation2.launch.py']
        )
    )
    
    # Patrol node
    patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        parameters=[default_patrol_config]
    )

    speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        name='speaker',
    )
    
    return launch.LaunchDescription([
        patrol_node,
        speaker_node,
    ])