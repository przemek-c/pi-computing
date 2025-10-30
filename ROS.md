# ROS2 Integration Plan for Mobile Robot Control

## Overview
This plan outlines integrating ROS2 (Humble) into the existing Python-based mobile robot control system on Raspberry Pi. The goal is to leverage ROS2 for high-level planning and odometry estimation, while STM32 MCU handles low-level motor control via UART. Key focus: Send desired linear/angular velocities from ROS2 to STM32, and read IMU/encoder data back for position/orientation estimation. Vision-based features (e.g., optical flow) will be added later. Path planning will use ROS2's advanced navigation stack (nav2) instead of custom Reeds-Shepp, for better flexibility in dynamic environments.

## Architecture
- **Raspberry Pi (ROS2 Host)**: Runs ROS2 nodes for path planning (using nav2), odometry fusion, and UART communication. Publishes `/cmd_vel` (desired velocities) and subscribes to `/odom` (estimated pose).
- **STM32 MCU**: Receives velocities via UART, uses PID for motor control (PWM via reliable timers). Sends IMU/encoder feedback back via UART.
- **Communication**: UART bridge node in ROS2 handles serial data exchange.
- **ROS2 Components**:
  - `nav2`: For advanced navigation, path planning (e.g., A*, Theta*), and following. Supports global/local planners, costmaps, and obstacle avoidance.
  - `robot_localization`: For fusing IMU/encoder data into odometry.
  - Custom nodes: UART bridge, odometry publisher.

## Prerequisites
- Hardware: Raspberry Pi 4/5 with Ubuntu 24.04, STM32 MCU connected via UART (/dev/ttyAMA0), IMU (e.g., MPU6050), wheel encoders.
- Software: ROS2 Humble installed (`sudo apt install ros-humble-desktop`).
- Libraries: `rclpy` (Python ROS2 client), `pyserial` for UART, existing uart module (remove reeds_shepp dependency).
- Dependencies: `sudo apt install python3-pip; pip install pandas numpy serial`.

## Step-by-Step Implementation

### 1. Set Up ROS2 Workspace
- Create a ROS2 workspace: `mkdir -p ~/ros2_ws/src; cd ~/ros2_ws; colcon build`.
- Source ROS2: Add `source /opt/ros/humble/setup.bash` to ~/.bashrc.
- Clone or create packages: Use `ros2 pkg create --build-type ament_python robot_control` for custom nodes.
- Install nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`.

### 2. Create UART Bridge Node
- Purpose: Subscribe to `/cmd_vel`, send desired velocities (linear.x, angular.z) to STM32 via UART. Receive IMU/encoder data and publish to `/odom`.
- Implementation:
  - Use `rclpy` to create a node.
  - UART format: Send as strings (e.g., "VEL:0.2,0.5" for linear/angular). Parse responses (e.g., "IMU:gyro_x,gyro_y,gyro_z;ENC:left,right").
  - Publish odometry: Use `nav_msgs/Odometry` with fused data.
- File: `~/ros2_ws/src/robot_control/robot_control/uart_bridge.py`.

### 3. Integrate Odometry Estimation
- Use `robot_localization` for fusing IMU and encoders.
- Configure: Create `ekf.yaml` for Extended Kalman Filter (fuse IMU for orientation, encoders for position).
- Node: Launch `robot_localization` with IMU topic (e.g., `/imu/data`) and encoder topic (e.g., `/encoders`).
- Publish to `/odom`: Estimated pose from fusion.

### 4. Path Planning and Navigation with nav2
- Replace Reeds-Shepp: Use nav2's planners (e.g., NavFn for global A*, DWB for local control) for more advanced path generation, supporting dynamic obstacles and costmaps.
- Node: Subscribe to goals (e.g., via `nav2_msgs/NavigateToPose`), generate path using nav2, publish to `/plan`.
- Integrate nav2: Use `nav2_bringup` for controller (publishes `/cmd_vel` to follow path). Configure planners in YAML (e.g., enable Theta* for smoother paths).
- Launch file: `navigation_launch.py` to start nav2 stack with custom params (e.g., robot footprint, max velocity).

### 5. Modify Existing Code
- Update `main_controller.py`: Remove `generate_path_segment` and `create_path_dataframe` (Reeds-Shepp related). Instead, publish navigation goals to ROS topics (e.g., `NavigateToPose` action).
- IMU/Encoders: Ensure STM sends data in ROS-compatible format (e.g., sensor_msgs/Imu for IMU).
- Velocity Handling: ROS `/cmd_vel` uses m/s and rad/s – scale as needed for STM (e.g., convert to cm/s for UART).

### 6. Testing and Validation
- Simulation: Use Gazebo (`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`) to test nav2 path planning without hardware.
- Hardware: Launch nodes, send goals via RViz, monitor `/cmd_vel` and `/odom`.
- Debug: Use `ros2 topic echo /cmd_vel` and `rqt` for visualization. Test path quality vs. Reeds-Shepp (nav2 handles non-holonomic constraints better).
- Metrics: Compare path smoothness, obstacle avoidance; tune PID on STM for velocity tracking.

## Challenges and Mitigations
- Latency: UART is serial – optimize baud rate (e.g., 115200).
- Synchronization: Use ROS timestamps for data alignment.
- nav2 Configuration: Tune costmaps and planners for your robot's kinematics (e.g., differential drive).
- Scaling: Test on short paths first; add vision later for better odometry.

## Next Steps
- Implement UART bridge node first.
- Test odometry fusion with IMU/encoders.
- Configure and launch nav2 for path planning.

This plan replaces Reeds-Shepp with nav2's advanced solutions for flexible, obstacle-aware navigation. Adjust based on hardware specifics.