# Autonomous Patrol Robot

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

An autonomous patrol robot system built with ROS 2 Navigation2 stack, featuring waypoint navigation, real-time image capture, and comprehensive sensor integration. The robot autonomously navigates through predefined waypoints in a simulated environment, captures images at each checkpoint, and provides status announcements.

## 🚀 Features

- **Autonomous Navigation**: Complete Navigation2 integration with AMCL localization and path planning
- **Waypoint Patrol**: Sequential navigation through configurable waypoints with automatic pose initialization
- **Image Capture**: Automatic image saving at each waypoint with meaningful filenames (relative timestamp + point index)
- **Multi-Sensor Integration**: LIDAR, IMU, and camera sensors for comprehensive perception
- **Gazebo Simulation**: Full physics-based simulation environment for testing and development
- **ROS 2 Services**: Custom service interfaces for text-to-speech announcements
- **Modular Architecture**: Clean package structure with separation of concerns

## 🛠️ Technology Stack

- **ROS 2 Humble**: Robot Operating System 2
- **Navigation2**: Autonomous navigation framework
- **Gazebo/Ignition**: Physics simulation engine
- **Python 3.10**: Core patrol logic and image processing
- **OpenCV**: Image processing and computer vision
- **URDF/XACRO**: Robot model description
- **AMCL**: Adaptive Monte Carlo Localization
- **RViz2**: 3D visualization tool

## 📸 Demo / Screenshots

<!-- Add screenshots or demo video links here -->
<!-- Example: ![Robot in Gazebo](docs/images/gazebo_sim.png) -->
<!-- Example: [Demo Video](https://youtube.com/watch?v=...) -->

## 📋 Prerequisites

### System Requirements
- Ubuntu 22.04 (Jammy) or compatible
- ROS 2 Humble Desktop
- Gazebo/Ignition Fortress (Garden is also supported)

### Required ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    python3-pip \
    python3-colcon-common-extensions
```

### Python Dependencies

```bash
pip3 install opencv-python cv-bridge numpy
```

### Additional Dependencies

If you encounter issues with `tf_transformations`, install:

```bash
pip3 install transforms3d
```

Or use the ROS 2 version:

```bash
sudo apt install ros-humble-tf-transformations
```

## 🚀 Quick Start

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd auto_patrol_robot
   ```

2. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the workspace**:
   ```bash
   cd autopatrol_ws
   colcon build --symlink-install
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

5. **Run the complete system**:
   ```bash
   # Terminal 1: Gazebo Simulation
   ros2 launch robot_description gazebo_sim.launch.py
   
   # Terminal 2: Navigation2
   ros2 launch robot_navigation2 navigation2.launch.py
   
   # Terminal 3: Autonomous Patrol
   ros2 launch autopatrol_robot autopatrol.launch.py
   ```

## 📁 Project Structure

```
autopatrol_ws/
├── src/
│   ├── robot_description/       # Robot URDF model, launch files, and Gazebo world
│   │   ├── urdf/robot/          # Robot model files (XACRO)
│   │   ├── launch/              # Launch files for simulation
│   │   ├── config/              # Configuration files (RViz, controllers)
│   │   └── world/                # Gazebo world files
│   ├── robot_navigation2/        # Navigation2 configuration and maps
│   │   ├── config/              # Navigation parameters
│   │   ├── launch/              # Navigation launch files
│   │   └── maps/                # Map files (YAML and PGM)
│   ├── autopatrol_robot/         # Autonomous patrol node and speaker service
│   │   ├── autopatrol_robot/    # Python package
│   │   ├── config/              # Patrol configuration
│   │   └── launch/              # Patrol launch files
│   └── autopatrol_interfaces/    # Custom service interfaces
│       └── srv/                  # Service definitions
├── media/                         # Captured images from patrol missions
│   └── README.md                 # Media folder documentation
├── build/                         # Build artifacts
├── install/                       # Installed packages
└── log/                           # Build logs
```

## 📖 Usage

### Gazebo Simulation

Launch the Gazebo simulation with the robot model:

```bash
ros2 launch robot_description gazebo_sim.launch.py
```

This spawns the robot with sensors (LIDAR, IMU, camera) and sets up ROS-Gazebo bridges.

### Navigation2

Start the Navigation2 stack with AMCL localization:

```bash
ros2 launch robot_navigation2 navigation2.launch.py
```

### Autonomous Patrol

Run the complete autonomous patrol system:

```bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

### Configuration

Configure patrol waypoints in `autopatrol_robot/config/patrol_config.yaml`:

```yaml
/patrol_node:
  ros__parameters:
    initial_point: [0.0, 0.0, 0.0]  # Initial robot pose [x, y, yaw]
    target_points: [                # Waypoints [x1, y1, yaw1, x2, y2, yaw2, ...]
      0.0,  0.0,   0.0,
      1.0,  2.0,   3.14,
     -4.5,  1.5,   1.57,
     -8.0, -5.0,   1.57,
      1.0, -5.0,   3.14
    ]
    img_save_path: ''  # Optional: custom path to save images (default: autopatrol_ws/media/)
```

**Image Saving:**
- Images are automatically saved to `autopatrol_ws/media/` by default
- Filename format: `time_{seconds}s_point_{index}.png` (e.g., `time_45s_point_2.png`)
- The timestamp is relative to the start of the patrol mission
- You can specify a custom path using the `img_save_path` parameter

## 🔧 Key Components

### Patrol Node
- Autonomous waypoint navigation using Navigation2
- Image capture at each waypoint with timestamped filenames
- Status announcements via text-to-speech service
- Configurable patrol routes
- Automatic media folder management (creates `autopatrol_ws/media/` if needed)
- Uses ROS 2 package discovery for reliable path resolution

### Robot Model
- Differential drive robot with LIDAR, IMU, and camera sensors
- URDF/XACRO-based model for easy customization
- Gazebo plugins for sensor simulation

### Navigation Stack
- AMCL for robust localization
- Navigation2 planners and controllers
- Custom map support
- Recovery behaviors

## 🐛 Troubleshooting

### Gazebo Not Starting
- Ensure Gazebo/Ignition is installed: `sudo apt install gazebo-fortress` or `gazebo-garden`
- Check that `ros_gz_sim` package is installed
- Verify world file exists: `autopatrol_ws/src/robot_description/world/custom_room.world`

### Navigation2 Not Localizing
- Make sure robot pose is initialized before navigation
- Check that map file exists and is valid
- Verify `/scan` topic is publishing LIDAR data
- Check RViz2 to see if robot appears on the map

### Patrol Node Not Working
- Ensure Navigation2 is fully active (wait for "Nav2 is ready" message)
- Check that robot pose is initialized before starting patrol
- Verify waypoints are within map bounds
- Check speaker service is running if using text-to-speech
- Verify camera topic `/camera/image_raw` is publishing images
- Check logs for image save path information if images aren't being saved

### Build Errors
- Make sure all dependencies are installed
- Clean build: `rm -rf build install log` then `colcon build --symlink-install`
- Check Python version compatibility (Python 3.10 recommended for Humble)

## 📝 Useful Commands

```bash
# View all topics
ros2 topic list

# Monitor robot pose
ros2 topic echo /amcl_pose

# Monitor navigation goal
ros2 topic echo /navigate_to_pose/_action/feedback

# Check Navigation2 status
ros2 service list | grep nav2

# View robot transforms
ros2 run tf2_ros tf2_echo map base_footprint

# View captured images
ls -lh autopatrol_ws/media/

# Monitor image capture
ros2 topic echo /camera/image_raw --once
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the Apache-2.0 License - see the LICENSE file for details.

## 👤 Author

**nayr**
- Email: ryan881028@gmail.com

## 🙏 Acknowledgments

- ROS 2 Navigation2 team for the excellent navigation framework
- Gazebo/Ignition team for the simulation platform
- ROS 2 community for continuous support and improvements
