# DS Node - ROS2 Data Acquisition Node

## Project Overview

A "DS Device" refers to a line of high-precision navigation sensors, such as those manufactured by PolyExplore Inc. These devices are typically combined GNSS (Global Navigation Satellite System) and INS (Inertial Navigation System) units that provide accurate position, velocity, attitude (orientation), and raw sensor data.

This "DS Node" is a ROS2 package that acts as a driver for this hardware. It connects to the device via Ethernet, parses its proprietary message formats (like Kalman, GnssHmr, RawIMU, etc.), and converts them into ROS2 standard message formats (like sensor_msgs::msg::NavSatFix and sensor_msgs::msg::Imu) for publishing on ROS2 topics.

## Features

- Supports communication with DS devices via Ethernet
- Parses multiple DS device message formats (Kalman, GnssHmr, RawIMU, SolutionStatus, etc.)
- Converts to ROS2 standard message formats (NavSatFix, Imu, PoseStamped, TwistStamped, etc.)
- Supports custom coordinate system transformations
- Provides extensive configuration options

## Supported Message Types

### Custom Message Types
- `ds_node::msg::Kalman` - Kalman filter data
- `ds_node::msg::GnssHmr` - GNSS high-precision measurement data
- `ds_node::msg::RawIMU` - Raw IMU data
- `ds_node::msg::SolutionStatus` - Solution status information
- `ds_node::msg::CompactNav` - Compact navigation data
- `ds_node::msg::EulerAttitude` - Euler angle attitude data
- `ds_node::msg::TimeSync` - Time synchronization data
- `ds_node::msg::Geoid` - Geoid data
- `ds_node::msg::CorrectedIMU` - Corrected IMU data
- `ds_node::msg::LeapSeconds` - Leap seconds information
- `ds_node::msg::NmeaGGA` - NMEA GGA format data
- `ds_node::msg::Dmi` - Distance measurement device data

### ROS2 Standard Message Types
- `sensor_msgs::msg::NavSatFix` - GPS positioning data
- `sensor_msgs::msg::Imu` - IMU data
- `geometry_msgs::msg::PoseStamped` - Pose data
- `geometry_msgs::msg::TwistStamped` - Velocity data
- `geometry_msgs::msg::AccelStamped` - Acceleration data

## Installation Instructions

### Prerequisites
- ROS2 Humble (or other compatible versions)
- CMake 3.8 or higher
- C++17 compatible compiler

### Compilation and Installation

1. Create and initialize ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

2. Clone the repository to the src directory of the workspace:
```bash
cd ~/ros2_ws/src
git clone [repository URL]  # if available
# or directly copy the ds_node directory to the src directory
```

3. Build the project:
```bash
cd ~/ros2_ws
colcon build --packages-select ds_node
```

4. Install dependencies (if needed):
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Running the Node

### Using Launch File

1. Source the ROS2 environment:
```bash
source install/setup.bash
```

2. Launch the node using the launch file:
```bash
ros2 launch ds_node ds_talker_launch.py
```

### Using RTCM Forwarder

If RTCM data forwarding is required, use the alternative launch file:
```bash
ros2 launch ds_node ds_talker_rtcm_forwarder_launch.py
```

## Configuration Options

The node's configuration parameters are located in the `param/ds_talker_params.yaml` file and can be modified as needed:

```yaml
/ds_ns:
  ds_node_talker:
    ros__parameters:
      use_sim_time: False  # Whether to use simulation time
      eth_enable: True     # Whether to enable Ethernet connection
      eth_server: "192.168.28.29"  # IP address of the DS device
      eth_port: "2829"     # Port number of the DS device
      ds_output: 0xFF      # Output control flag
```

## Data Conversion Functions

The node provides various coordinate system transformations and data format conversion functions, including:

- WGS84 to NAD83 coordinate system conversion
- Geodetic to ECEF coordinate system conversion
- ECEF to geodetic coordinate system conversion
- Quaternion to Euler angle conversion
- Conversion between various message formats

## Contribution Guidelines

Issue reports and feature requests are welcome. For code contributions, please submit pull requests.

## License

This project is licensed under the Apache License 2.0 - see below for details.

```
Copyright 2025 DSTD

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

For the full license text, please see the [LICENSE.txt](LICENSE.txt) file in this repository.

---

# DS Node - ROS2 Data Acquisition Node

## Project Overview

DS Node is a ROS2 node package designed to receive data from DS devices (possibly sensors or navigation systems) and convert it into ROS2 standard message formats for publishing on ROS2 topics. The node supports parsing and publishing various data types, including position, velocity, acceleration, attitude, and raw sensor data.

## Features

- Supports communication with DS devices via Ethernet
- Parses multiple DS device message formats (Kalman, GnssHmr, RawIMU, SolutionStatus, etc.)
- Converts to ROS2 standard message formats (NavSatFix, Imu, PoseStamped, TwistStamped, etc.)
- Supports custom coordinate system transformations
- Provides extensive configuration options

## Supported Message Types

### Custom Message Types
- `ds_node::msg::Kalman` - Kalman filter data
- `ds_node::msg::GnssHmr` - GNSS high-precision measurement data
- `ds_node::msg::RawIMU` - Raw IMU data
- `ds_node::msg::SolutionStatus` - Solution status information
- `ds_node::msg::CompactNav` - Compact navigation data
- `ds_node::msg::EulerAttitude` - Euler angle attitude data
- `ds_node::msg::TimeSync` - Time synchronization data
- `ds_node::msg::Geoid` - Geoid data
- `ds_node::msg::CorrectedIMU` - Corrected IMU data
- `ds_node::msg::LeapSeconds` - Leap seconds information
- `ds_node::msg::NmeaGGA` - NMEA GGA format data
- `ds_node::msg::Dmi` - Distance measurement device data

### ROS2 Standard Message Types
- `sensor_msgs::msg::NavSatFix` - GPS positioning data
- `sensor_msgs::msg::Imu` - IMU data
- `geometry_msgs::msg::PoseStamped` - Pose data
- `geometry_msgs::msg::TwistStamped` - Velocity data
- `geometry_msgs::msg::AccelStamped` - Acceleration data

## Installation Instructions

### Prerequisites
- ROS2 Humble (or other compatible versions)
- CMake 3.8 or higher
- C++17 compatible compiler

### Compilation and Installation

1. Create and initialize ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

2. Clone the repository to the src directory of the workspace:
```bash
cd ~/ros2_ws/src
git clone [repository URL]  # if available
# or directly copy the ds_node directory to the src directory
```

3. Build the project:
```bash
cd ~/ros2_ws
colcon build --packages-select ds_node
```

4. Install dependencies (if needed):
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Running the Node

### Using Launch File

1. Source the ROS2 environment:
```bash
source ~/ros2_ws/install/setup.bash
```

2. Launch the node using the launch file:
```bash
ros2 launch ds_node ds_talker_launch.py
```

### Using RTCM Forwarder

If RTCM data forwarding is required, use the alternative launch file:
```bash
ros2 launch ds_node ds_talker_rtcm_forwarder_launch.py
```

## Configuration Options

The node's configuration parameters are located in the `param/ds_talker_params.yaml` file and can be modified as needed:

```yaml
/ds_ns:
  ds_node_talker:
    ros__parameters:
      use_sim_time: False  # Whether to use simulation time
      eth_enable: True     # Whether to enable Ethernet connection
      eth_server: "192.168.28.29"  # IP address of the DS device
      eth_port: "2829"     # Port number of the DS device
      ds_output: 0xFF      # Output control flag
```

## Data Conversion Functions

The node provides various coordinate system transformations and data format conversion functions, including:

- WGS84 to NAD83 coordinate system conversion
- Geodetic to ECEF coordinate system conversion
- ECEF to geodetic coordinate system conversion
- Quaternion to Euler angle conversion
- Conversion between various message formats

## Contribution Guidelines

Issue reports and feature requests are welcome. For code contributions, please submit pull requests.

## Tests
TO DO
