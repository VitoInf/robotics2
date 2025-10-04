# Multi-Robot Simulation

A ROS2 Humble-based multi-robot formation control simulation using TurtleBot3 robots in Gazebo. This project implements a leader-follower formation with obstacle avoidance capabilities using Docker containerization.

## Features

- **Multi-robot formation control**: One leader robot and two follower robots
- **Obstacle avoidance**: Integrated obstacle detection and avoidance using sensor data
- **Trajectory generation**: Cubic spline trajectory generation for smooth path planning
- **SLAM integration**: Real-time mapping using SLAM Toolbox
- **Dockerized environment**: Containerized setup for easy deployment
- **RViz visualization**: Real-time visualization of robots, sensors, and map data
- **Data logging & analysis**: Real-time monitoring and post-processing tools for performance evaluation

## System Architecture

The simulation consists of three main components:

### 1. Leader Robot
- Follows a predefined cubic spline trajectory
- Uses LiDAR sensor for obstacle detection and avoidance
- Implements PID controller for trajectory tracking

### 2. Follower Robots (Left & Right)
- Maintain formation relative to the leader robot
- Use ultrasonic sensors for obstacle avoidance
- Implement formation control with configurable offsets

### 3. Trajectory Generator
- Generates smooth cubic spline trajectories through waypoints
- Publishes desired poses for the leader robot

## Prerequisites

- Ubuntu 20.04/22.04 with WSL2 (for Windows users)
- Docker installed and configured
- X11 forwarding enabled for GUI applications

## Installation & Setup

### 1. Clone and Prepare Files

```bash
# Extract the provided files.zip to your WSL home directory
unzip files.zip
cp -r extracted_files/* ~/
cd ~/
```

### 2. Build and Run Container

```bash
# Make the script executable and run
chmod +x create_container.sh
bash create_container.sh
```

This script will:
- Build the Docker image with ROS2 Humble and dependencies
- Create and start the container
- Build the workspace
- Launch Gazebo, RViz, and the multi-robot simulation

### 3. Start Trajectory Generator and Data Logger

In a new WSL terminal:

```bash
chmod +x run_traj.sh
bash run_traj.sh
```

The script will prompt you to choose which robot to analyze:
- **Option 1**: Leader (default) - Tracks trajectory following performance
- **Option 2**: Left Follower - Monitors formation control and obstacle avoidance
- **Option 3**: Right Follower - Monitors formation control and obstacle avoidance

The data logger will:
- Display real-time plots of robot performance
- Save data to `.npz` files for post-processing
- Monitor position errors, velocity commands, and obstacle distances

## Package Structure

```
multirobot_ws/
├── src/
│   └── multirobot_sim/
│       ├── launch/                 # Launch files
│       │   ├── multirobot.launch.py
│       │   └── robot_spawner.launch.py
│       ├── models/                 # Robot and obstacle models
│       │   ├── left_follower/
│       │   ├── right_follower/
│       │   └── unit_cylinder/
│       ├── multirobot_sim/         # Python source code
│       │   ├── leader_controller.py
│       │   ├── follower_controller.py
│       │   ├── trajectory_generator.py
│       │   └── data_logger.py
│       ├── scripts/                # Analysis tools
│       │   └── plot_analysis.py
│       ├── rviz/                   # RViz configuration
│       │   └── multirobot.rviz
│       └── worlds/                 # Gazebo world files
│           └── field.world
```

## Robot Configuration

### Leader Robot
- **Position**: (9.5, 0.0) with yaw = π
- **Sensors**: LiDAR scanner
- **Controller**: PID with obstacle avoidance
- **Trajectory**: Cubic spline through predefined waypoints

### Follower Robots

**Left Follower:**
- Offset: (-2.0, 2.0) relative to leader
- Position: (11.5, -2.0)

**Right Follower:**
- Offset: (-2.0, -2.0) relative to leader
- Position: (11.5, 2.0)

**Both followers:**
- Sensors: Ultrasonic sensors
- Controller: Formation control with obstacle avoidance

## Controller Parameters

### PID Gains
- Linear velocity: Kp=2.25, Ki=0.0, Kd=0.1
- Angular velocity: Kp=1.5, Ki=0.0, Kd=0.05

### Obstacle Avoidance
- **Leader**:
  - Safe distance: 0.5m
  - Detection range: 1.2m
  - Avoidance distance: 1.5-2.0m travel distance during maneuver
- **Followers**:
  - Safe distance: 0.3m
  - Detection range: 0.8m
  - Avoidance distance: 1.0-1.5m travel distance during maneuver

<img width="1204" height="785" alt="image" src="https://github.com/user-attachments/assets/c26cf90f-cefa-45f0-85e5-29de8fa3aa1b" />


## Usage Instructions

### 1. Basic Operation
1. Run the container setup script: `bash create_container.sh`
2. Wait for Gazebo and RViz to load completely
3. In a new terminal, start trajectory generation and logging: `bash run_traj.sh`
4. Select the robot you want to monitor (Leader, Left Follower, or Right Follower)
5. Observe the robots following the formation and real-time performance plots

### 2. Adding Obstacles
1. In Gazebo, go to the "Insert" tab
2. Select "unit cylinder" model
3. Drag and drop into the simulation environment
4. Watch robots perform obstacle avoidance maneuvers

### 3. Monitoring Performance

**Real-time Monitoring:**
- RViz: Monitor robot positions, sensor data, and SLAM mapping
- Terminal output: View controller status and debugging information
- Gazebo: Visual simulation environment
- Live plots: Real-time graphs showing trajectory, errors, velocities, and obstacle distances

**Post-Processing Analysis:**

After data collection, analyze saved logs:

```bash
# Inside the container or with Python 3 installed
python3 multirobot_ws/src/multirobot_sim/scripts/plot_analysis.py robot_log_YYYYMMDD_HHMMSS.npz
```

This generates comprehensive performance reports including:
- Trajectory comparison (actual vs desired)
- Position and orientation errors over time
- Velocity command profiles
- Obstacle avoidance analysis
- Statistical summaries

Options:
```bash
# Custom output filename
python3 plot_analysis.py log_file.npz -o custom_report.png

# Statistics only (no plots)
python3 plot_analysis.py log_file.npz --no-plot
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Leader robot velocity commands |
| `/odom` | nav_msgs/Odometry | Leader robot odometry |
| `/scan` | sensor_msgs/LaserScan | Leader LiDAR data |
| `/desired_trajectory` | geometry_msgs/Pose2D | Target trajectory for leader |
| `/{robot_name}/cmd_vel` | geometry_msgs/Twist | Follower velocity commands |
| `/{robot_name}/odom` | nav_msgs/Odometry | Follower odometry |
| `/{robot_name}/ultrasonic_sensor` | sensor_msgs/LaserScan | Follower ultrasonic data |
| `/map` | nav_msgs/OccupancyGrid | SLAM-generated map |

## Troubleshooting

### Common Issues

**Container fails to start**
- Ensure Docker is running and X11 forwarding is enabled
- Check that WSL DISPLAY variable is set: `export DISPLAY=:0`

**Gazebo doesn't open**
- Verify X11 forwarding: `xhost +local:`
- Check Docker container has GUI access

**Robots don't move**
- Ensure trajectory generator is running
- Check ROS topic connections: `ros2 topic list`

**Build failures**
- Clean workspace: `rm -rf build/ install/ log/`
- Rebuild: `colcon build`

**Plots don't appear**
- Verify matplotlib backend supports display
- Check X11 forwarding is working properly

### Performance Optimization

- Reduce Gazebo physics update rate if simulation is slow
- Adjust PID gains for smoother control
- Modify obstacle avoidance parameters for different environments

## Advanced Configuration

### Modifying Formation

Edit the launch file parameters in `multirobot.launch.py`:

```python
'offset_x': -2.0,  # Distance behind leader
'offset_y': 2.0    # Lateral offset from leader
```

### Custom Trajectories

Modify waypoints in `trajectory_generator.py`:

```python
self.waypoints = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
```

### Custom Time

Modify time in `trajectory_generator.py`:

```python
self.times = np.array([0.0, 15.0, 30.0, 40.0])
```

### Sensor Configuration

Adjust sensor parameters in the robot SDF model files for different detection ranges and accuracies.

### Data Logger Configuration

Customize logging parameters when starting the data logger:

```bash
# Inside container
ros2 run multirobot_sim data_logger --ros-args \
    -p robot_name:=left_follower \
    -p log_file:=custom_name.npz \
    -p plot_live:=false
```

## Dependencies

- ROS2 Humble
- Gazebo Classic
- TurtleBot3 packages
- SLAM Toolbox
- tf_transformations
- Docker
- Python 3 with:
  - numpy
  - matplotlib
  - rclpy

## License

This project is provided as-is for educational and research purposes.

## Contributing

Feel free to submit issues and enhancement requests. When contributing:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Support

For questions and support, please refer to the documentation or create an issue in the repository.
