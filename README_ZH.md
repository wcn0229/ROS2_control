# DS Node - ROS2 数据采集节点

## 项目概述

DS Node 是一个 ROS2 节点包，用于从 DS 设备接收数据，并将其转换为 ROS2 标准消息格式发布到 ROS2 主题上。该节点支持多种数据类型的解析和发布，包括位置、速度、加速度、姿态和原始传感器数据等。

## 功能特性

- 支持通过以太网连接与 DS 设备通信
- 解析多种 DS 设备消息格式（Kalman、GnssHmr、RawIMU、SolutionStatus 等）
- 转换为 ROS2 标准消息格式（NavSatFix、Imu、PoseStamped、TwistStamped 等）
- 支持自定义坐标系转换
- 提供丰富的配置选项

## 支持的消息类型

### 自定义消息类型
- `ds_node::msg::Kalman` - 卡尔曼滤波数据
- `ds_node::msg::GnssHmr` - GNSS 高精度测量数据
- `ds_node::msg::RawIMU` - 原始 IMU 数据
- `ds_node::msg::SolutionStatus` - 解算状态信息
- `ds_node::msg::CompactNav` - 紧凑导航数据
- `ds_node::msg::EulerAttitude` - 欧拉角姿态数据
- `ds_node::msg::TimeSync` - 时间同步数据
- `ds_node::msg::Geoid` - 大地水准面数据
- `ds_node::msg::CorrectedIMU` - 校正后的 IMU 数据
- `ds_node::msg::LeapSeconds` - 闰秒信息
- `ds_node::msg::NmeaGGA` - NMEA GGA 格式数据
- `ds_node::msg::Dmi` - 距离测量设备数据

### ROS2 标准消息类型
- `sensor_msgs::msg::NavSatFix` - GPS 定位数据
- `sensor_msgs::msg::Imu` - IMU 数据
- `geometry_msgs::msg::PoseStamped` - 位姿数据
- `geometry_msgs::msg::TwistStamped` - 速度数据
- `geometry_msgs::msg::AccelStamped` - 加速度数据

## 安装说明

### 前提条件
- ROS2 Humble (或其他兼容版本)
- CMake 3.8 或更高版本
- C++17 兼容编译器

### 编译安装

1. 创建并初始化 ROS2 工作空间：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

2. 克隆仓库到工作空间的 src 目录：
```bash
cd ~/ros2_ws/src
git clone [仓库地址]  # 如果有远程仓库
# 或者直接复制 ds_node 目录到 src 目录
```

3. 编译项目：
```bash
cd ~/ros2_ws
colcon build --packages-select ds_node
```

4. 安装依赖（如果需要）：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 运行方式

### 使用启动文件运行

1. 加载 ROS2 环境：
```bash
source install/setup.bash
```

2. 使用 launch 文件启动节点：
```bash
ros2 launch ds_node ds_talker_launch.py
```

### 使用 RTCM 转发器启动

如果需要转发 RTCM 数据，可以使用另一个启动文件：
```bash
ros2 launch ds_node ds_talker_rtcm_forwarder_launch.py
```

## 配置选项

节点的配置参数位于 `param/ds_talker_params.yaml` 文件中，可以根据需要修改：

```yaml
/ds_ns:
  ds_node_talker:
    ros__parameters:
      use_sim_time: False  # 是否使用模拟时间
      eth_enable: True     # 是否启用以太网连接
      eth_server: "192.168.28.29"  # DS 设备的 IP 地址
      eth_port: "2829"     # DS 设备的端口号
      ds_output: 0xFF      # 输出控制标志
```

## 数据转换功能

该节点提供了多种坐标系转换和数据格式转换功能，包括：

- WGS84 到 NAD83 坐标系转换
- 大地坐标系到 ECEF 坐标系转换
- ECEF 坐标系到大地坐标系转换
- 四元数与欧拉角转换
- 各种消息格式之间的转换

## 贡献指南

欢迎提交问题报告和功能请求。如需贡献代码，请提交拉取请求。

## 许可证

TODO: 许可证声明