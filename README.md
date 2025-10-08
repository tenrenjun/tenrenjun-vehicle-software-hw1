# first_hw ROS Package
## 项目简介
first_hw 是一个基于 ROS 的车辆传感器数据处理作业，包含点云、图像、IMU、里程计等多种节点，演示了多传感器数据的采集与处理。

## 目录结构
- src：主要 C++ 源码
  - pcl_node.cpp：点云处理
  - show_color_and_depth_node.cpp：显示颜色和深度
  - imu_node.cpp：IMU 传感器
  - odom.cpp：里程计
  - opencv_image_node.cpp：图像处理
  - imageConverter.hpp：图像转换头文件
- first_hw.launch：一键启动所有节点
- CMakeLists.txt、package.xml：ROS 包配置

## 依赖
- ROS (建议 Kinetic 或更高版本)
- OpenCV
- PCL
- ROS 依赖包：
  - roscpp
  - rospy
  - std_msgs
  - pcl_ros
  - pcl_conversions
  - image_transport
  - cv_bridge


## 编译方法
在 ROS 工作空间根目录下执行：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 启动方法
编译完成后，使用如下命令启动所有节点：
```bash
roslaunch first_hw first_hw.launch
```

## 节点说明
- **pcl_node**：处理点云数据，订阅传感器点云话题并进行滤波、分割等处理。
- **show_color_and_depth_node**：显示颜色和深度信息，融合 RGB 与深度图像。
- **imu_node**：采集 IMU 数据，发布加速度、角速度等信息。
- **odom_node**：里程计数据处理，计算车辆位姿。
- **opencv_image_node**：图像处理（如有），实现图像采集与转换。

