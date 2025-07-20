# VESC Differential Drive Odometry Package

## Overview

A high-performance C++ ROS2 package that provides real-time differential drive odometry for robots using VESC motor controllers. The package reads tachometer data from VESC controllers via CAN bus and calculates precise robot odometry including position, orientation, and velocities.

## Features

- **High Performance**: Optimized C++ implementation for real-time operation
- **Real-time Odometry**: Calculates robot position, orientation, and velocities from VESC tachometer data
- **Direct CAN Communication**: Uses Linux SocketCAN for low-latency communication
- **Configurable Parameters**: Easy configuration via YAML parameter files
- **TF Broadcasting**: Publishes odom → base_link transform for navigation stack integration
- **Multi-topic Publishing**: Comprehensive data output for monitoring and debugging
- **Thread-safe Operation**: Mutex-protected data access for reliable multi-threaded operation

## Hardware Requirements

- Robot with differential drive configuration
- 2x VESC motor controllers with CAN bus capability
- CAN interface (e.g., USB-to-CAN adapter)
- Linux system with SocketCAN support

## Default Robot Configuration

This package is pre-configured for:
- **Wheel diameter**: 355.6mm
- **Wheel separation**: 370mm (distance between wheel centers)
- **Tachometer pulses per revolution**: 23 (mechanical pulses)
- **Left VESC ID**: 28 (CAN ID: 0x1B1C)
- **Right VESC ID**: 46 (CAN ID: 0x1B2E)
- **CAN Interface**: can0 at 500kbps
- **Publish Rate**: 50Hz

## Installation

### 1. Prerequisites
```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-tf2-tools

# Install CAN utilities
sudo apt install can-utils
```

### 2. Build the Package
```bash
cd ~/robot_ws
colcon build --packages-select diff_vesc_can_ros2_pkg_cpp
source install/setup.bash
```

## CAN Bus Setup

### 1. Configure CAN Interface
```bash
# Bring up CAN interface
sudo ip link set can0 up type can bitrate 500000

# Verify CAN interface is up
ip link show can0
```

### 2. Verify CAN Communication
```bash
# Monitor CAN traffic
candump can0

# You should see messages like:
# can0  1B1C   [8]  XX XX XX XX XX XX XX XX
# can0  1B2E   [8]  XX XX XX XX XX XX XX XX
```

## Usage

### Basic Launch
```bash
ros2 launch diff_vesc_can_ros2_pkg_cpp vesc_odometry.launch.py
```

### Custom Parameters
```bash
ros2 launch diff_vesc_can_ros2_pkg_cpp vesc_odometry.launch.py \
    wheel_diameter:=0.360 \
    wheel_separation:=0.380 \
    tachometer_pulses_per_rev:=24 \
    left_vesc_id:=10 \
    right_vesc_id:=20 \
    publish_rate:=100.0
```

### Direct Node Execution
```bash
ros2 run diff_vesc_can_ros2_pkg_cpp vesc_odometry_node
```

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | 50Hz | Complete robot odometry with position, orientation, and velocities |
| `/left_wheel_distance` | `std_msgs/Float64` | 50Hz | Cumulative distance traveled by left wheel (meters) |
| `/right_wheel_distance` | `std_msgs/Float64` | 50Hz | Cumulative distance traveled by right wheel (meters) |
| `/wheel_velocities` | `geometry_msgs/Twist` | 50Hz | Individual wheel velocities (left=linear.x, right=linear.y) |
| `/tf` | `tf2_msgs/TFMessage` | 50Hz | Transform from odom to base_link frame |

## Configuration Parameters

### Robot Physical Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_diameter` | double | 0.3556 | Wheel diameter in meters |
| `wheel_separation` | double | 0.370 | Distance between wheel centers in meters |
| `tachometer_pulses_per_rev` | int | 23 | Mechanical pulses per wheel revolution |

### VESC Configuration
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `left_vesc_id` | int | 28 | Left wheel VESC controller ID |
| `right_vesc_id` | int | 46 | Right wheel VESC controller ID |

### System Configuration
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `can_interface` | string | "can0" | CAN interface name |
| `publish_rate` | double | 50.0 | Odometry publish rate in Hz |

## Monitoring and Debugging

### Check Odometry Data
```bash
# View complete odometry
ros2 topic echo /odom

# Monitor publishing rate
ros2 topic hz /odom

# Check wheel distances
ros2 topic echo /left_wheel_distance
ros2 topic echo /right_wheel_distance
```

### Monitor Robot Movement
```bash
# View wheel velocities
ros2 topic echo /wheel_velocities

# Visualize in RViz
rviz2 -d $(ros2 pkg prefix diff_vesc_can_ros2_pkg_cpp)/share/diff_vesc_can_ros2_pkg_cpp/rviz/odometry.rviz
```

### TF Tree Inspection
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

## Technical Details

### CAN Message Processing
- **Message Type**: VESC STATUS_5 (contains tachometer data)
- **CAN ID Calculation**: `CAN_ID = 0x1B00 + VESC_ID`
- **Data Format**: Tachometer value in bytes 0-3 (big-endian int32)
- **Tachometer Conversion**: Electrical → Mechanical (6:1 ratio)

### Odometry Calculations
- **Kinematics Model**: Standard differential drive kinematics
- **Distance Calculation**: `distance = tachometer_pulses × (π × wheel_diameter) / pulses_per_rev`
- **Velocity Calculation**: `velocity = distance_change / time_delta`
- **Position Integration**: Arc-based motion model for accurate curved path tracking

### Performance Characteristics
- **Real-time Operation**: 50Hz default with sub-millisecond processing
- **Low CPU Usage**: ~1-2% CPU on modern systems
- **Memory Efficient**: <10MB RAM usage
- **Thread Safety**: Mutex-protected shared data structures

## Troubleshooting

### CAN Interface Issues
```bash
# Check if CAN interface exists
ip link show | grep can

# Verify interface is UP
ip link show can0

# Reset CAN interface if needed
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

### No Tachometer Data
```bash
# Check CAN traffic
candump can0 | grep -E "(1B1C|1B2E)"

# Verify VESC IDs match your hardware
# Expected CAN IDs: 0x1B1C (VESC 28), 0x1B2E (VESC 46)
```

### Node Not Publishing
```bash
# Check node is running
ros2 node list | grep vesc_odometry_node

# Verify topics exist
ros2 topic list | grep -E "(odom|wheel)"

# Check for errors in logs
ros2 launch diff_vesc_can_ros2_pkg_cpp vesc_odometry.launch.py --ros-args --log-level DEBUG
```

### Performance Issues
- Reduce `publish_rate` if system is overloaded
- Check system load: `htop` or `top`
- Monitor CAN bus utilization: `canbusload can0@500000`

## Integration with Navigation Stack

This package is designed to work seamlessly with ROS2 Navigation Stack:

```bash
# Example navigation launch
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    map:=/path/to/map.yaml \
    params_file:=/path/to/nav2_params.yaml
```

The odometry data and TF transforms are automatically available to navigation nodes.

## Calibration

### Wheel Diameter Calibration
1. Mark a point on the wheel
2. Measure one complete revolution distance
3. Calculate: `diameter = measured_distance / π`

### Wheel Separation Calibration
1. Command robot to rotate 360°
2. Measure actual rotation
3. Adjust `wheel_separation` proportionally

### Tachometer Pulses Calibration
1. Manually rotate wheel one complete revolution
2. Count tachometer pulses in CAN messages
3. Update `tachometer_pulses_per_rev` parameter

## License

MIT License

## Support

For issues and questions:
- GitHub Issues: Create an issue in the repository
- Email: maikelborys@gmail.com

## Version History

- **v1.0.0**: Initial C++ implementation with full VESC CAN support
