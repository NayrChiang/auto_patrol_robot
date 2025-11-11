# Ch 6.4
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # URDF path - using Fortress compatible version
    urdf_package_path = get_package_share_directory('robot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'robot/robot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # 1) CLI arg for model path
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='Model File Path'
    )
    
    # 2) CLI arg for world path
    action_declare_arg_world_path = launch.actions.DeclareLaunchArgument(
        name='world',
        default_value=str(default_gazebo_world_path),
        description='Gazebo World File Path'
    )

    # 3) Process xacro -> URDF string
    robot_description_cmd = launch.substitutions.Command(
        ['xacro ', LaunchConfiguration('model')]
    )

    # 4) Robot State Publisher consumes the param (and sim time)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str),
            'use_sim_time': True
        }],
        output='screen'
    )

    # 5) Launch Gazebo Fortress
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']
        ),
        launch_arguments=[
            ('gz_args', ['-v 2 -r ', LaunchConfiguration('world')])
        ]
    )

    # 6) Spawn robot in Gazebo from the *parameter* (not topic)
    action_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-param', 'robot_description',     # <-- changed from -topic
            # Optional pose at spawn:
            # '-x', '0', '-y', '0', '-z', '0.02'
        ],
        parameters=[{
            'robot_description': ParameterValue(robot_description_cmd, value_type=str)
        }],
        output='screen'
    )

    # 7) ROS 2 <-> Gazebo bridges
    action_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Drive + odom + tf + clock
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',

            # Joint states (Gazebo -> ROS)
            '/model/robot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',

            # Sensors (Gazebo -> ROS)
            '/model/robot/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/robot/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/model/robot/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        remappings=[
            # Standard ROS names
            ('/model/robot/cmd_vel', '/cmd_vel'),
            ('/model/robot/odometry', '/odom'),
            ('/model/robot/tf', '/tf'),
            ('/model/robot/joint_state', '/joint_states'),

            # Sensor topic remaps
            ('/model/robot/lidar/scan', '/scan'),
            ('/model/robot/imu', '/imu'),
            ('/model/robot/camera/image_raw', '/camera/image_raw'),
        ],

        output='screen'
    )

    # 8) Small delay so Gazebo is up before spawning/bridging (optional)
    delayed_spawn = TimerAction(period=1.0, actions=[action_spawn_entity, action_ros_gz_bridge])

    # 9) Launch RViz2
    action_launch_rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('robot_description'), 'config', 'robot_config.rviz')]
        )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_declare_arg_world_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        delayed_spawn,
        # action_launch_rviz,
    ])