# ZNYQ_RV_ws TF树结构说明文档

## 概述

本文档详细说明了RoboMaster视觉系统中的TF（Transform）树结构、各坐标系的定义以及变换关系。TF树是ROS2中用于管理不同坐标系之间变换关系的重要机制，在本项目中用于统一处理相机检测、目标跟踪和弹道计算中的坐标转换。

## TF树结构概览

```
odom (世界坐标系)
  └── gimbal_link (云台坐标系)
      └── camera_link (相机物理坐标系)
          └── camera_optical_frame (相机光学坐标系)
```

## 各坐标系详解

### 1. odom坐标系

- **坐标系名称**：`odom`
- **定义**：世界坐标系，以云台中心为原点的惯性系
- **作用**：
  - 作为整个系统的根坐标系
  - 所有目标位置最终都转换到这个坐标系
  - 提供稳定的参考系用于目标跟踪
- **来源**：
  - URDF中定义为根链接
  - 串口驱动发布的TF变换的父坐标系
- **相关代码**：
  ```cpp
  // rm_serial_driver/src/rm_serial_driver.cpp:143
  t.header.frame_id = "odom";
  t.child_frame_id = "gimbal_link";
  ```

### 2. gimbal_link坐标系

- **坐标系名称**：`gimbal_link`
- **定义**：云台坐标系，随云台运动
- **作用**：
  - 表示云台的当前姿态
  - 作为相机和弹道计算的参考系
  - 连接世界坐标系和相机坐标系
- **变换来源**：
  - 由`rm_serial_driver`根据接收到的云台角度数据发布
  - 动态变换，随云台运动实时更新
- **相关代码**：
  ```cpp
  // rm_serial_driver/src/rm_serial_driver.cpp:145-147
  tf2::Quaternion q;
  q.setRPY(packet.roll, packet.pitch, packet.yaw);
  t.transform.rotation = tf2::toMsg(q);
  ```

### 3. camera_link坐标系

- **坐标系名称**：`camera_link`
- **定义**：相机物理安装位置坐标系
- **作用**：
  - 表示相机在云台上的物理安装位置
  - 提供相机机械安装的参考点
- **变换关系**：
  - 相对于gimbal_link有固定的偏移
  - 默认偏移：x=0.08m, y=0, z=0.065m
- **配置参数**：
  ```yaml
  # rm_vision/rm_vision_bringup/config/launch_params.yaml
  odom2camera:
    xyz: '"0.08 0.0  0.065"'
    rpy: '"0.0  0.0  0.0"'
  ```

### 4. camera_optical_frame坐标系

- **坐标系名称**：`camera_optical_frame`
- **定义**：相机光学坐标系
- **作用**：
  - 图像处理的参考系
  - 所有装甲板检测都在这个坐标系下进行
  - 符合相机光学成像的坐标系约定
- **变换关系**：
  - 相对于camera_link有固定的旋转变换
  - 绕X轴-90°，绕Z轴-90°（符合ROS相机坐标系约定）
- **相关代码**：
  ```xml
  <!-- rm_gimbal_description/urdf/rm_gimbal.urdf.xacro -->
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
    <parent link="camera_link" />
    <child link="camera_optical_frame" />
  </joint>
  ```

## TF变换的发布和订阅关系

### 发布者（TF Broadcasters）

#### 1. rm_serial_driver
- **发布变换**：`odom` → `gimbal_link`
- **数据来源**：串口接收的云台角度（roll, pitch, yaw）
- **变换类型**：动态变换（随云台运动实时更新）
- **发布频率**：与串口数据接收频率一致
- **相关代码位置**：
  - 头文件：`rm_serial_driver/include/rm_serial_driver/rm_serial_driver.hpp:79`
  - 实现：`rm_serial_driver/src/rm_serial_driver.cpp:36, 148`

#### 2. robot_state_publisher
- **发布变换**：
  - `gimbal_link` → `camera_link`（静态）
  - `camera_link` → `camera_optical_frame`（静态）
- **数据来源**：URDF文件描述
- **变换类型**：静态变换（系统启动时确定，后续不变）
- **配置文件**：`rm_gimbal_description/urdf/rm_gimbal.urdf.xacro`

### 订阅者（TF Listeners）

#### 1. armor_tracker
- **监听变换**：装甲板位置从相机坐标系到目标坐标系的变换
- **目标坐标系**：`odom`（可配置）
- **用途**：
  - 将检测到的装甲板位置转换到世界坐标系
  - 在统一坐标系下进行目标跟踪和预测
- **特殊机制**：使用tf2_message_filter确保变换可用时才处理数据
- **相关代码位置**：
  - 头文件：`rm_auto_aim/armor_tracker/include/armor_tracker/tracker_node.hpp:65`
  - 实现：`rm_auto_aim/armor_tracker/src/tracker_node.cpp:160, 215`

#### 2. projectile_motion
- **监听变换**：从射手坐标系到目标坐标系的变换
- **射手坐标系**：`shooter_frame_`（默认为`shooter_link`）
- **用途**：
  - 获取当前云台角度
  - 计算弹道时进行坐标转换
- **相关代码位置**：
  - 头文件：`rm_auto_aim/projectile_motion/include/projectile_motion/projectile_motion_node.hpp:85-86`
  - 实现：`rm_auto_aim/projectile_motion/src/projectile_motion_node.cpp:77, 130`

## 数据流向和变换链

### 装甲板检测流程的坐标变换

1. **检测阶段**：
   - 在`camera_optical_frame`下检测装甲板
   - 输出：装甲板在相机坐标系下的3D位置

2. **坐标转换链**：
   ```
   camera_optical_frame 
   → camera_link 
   → gimbal_link 
   → odom
   ```

3. **目标跟踪**：
   - 在`odom`坐标系下进行目标跟踪和预测
   - 输出：目标在世界坐标系下的状态估计

### 弹道计算的坐标变换

1. **获取目标位置**：
   - 从`odom`坐标系获取目标位置
   - 来源：armor_tracker的输出

2. **变换到射手系**：
   ```
   odom 
   → gimbal_link 
   → shooter_frame_
   ```

3. **弹道解算**：
   - 在射手坐标系下计算射击角度
   - 输出：pitch和yaw角度指令

## 关键配置参数

### 1. 相机安装位置配置
- **文件**：`rm_vision/rm_vision_bringup/config/launch_params.yaml`
- **参数**：
  ```yaml
  odom2camera:
    xyz: '"0.08 0.0  0.065"'  # 相机相对于云台的偏移（米）
    rpy: '"0.0  0.0  0.0"'     # 相机相对于云台的旋转（弧度）
  ```

### 2. 跟踪器目标坐标系配置
- **文件**：`rm_vision/rm_vision_bringup/config/node_params.yaml`
- **参数**：
  ```yaml
  /armor_tracker:
    target_frame: odom  # 跟踪器输出的目标坐标系
  ```

### 3. 弹道计算目标坐标系配置
- **文件**：`rm_auto_aim/projectile_motion/config/projectile_motion.yaml`
- **参数**：
  ```yaml
  target_frame: "gimbal_link"  # 弹道计算的参考坐标系
  ```

## URDF坐标系定义

完整的坐标系定义在`rm_gimbal_description/urdf/rm_gimbal.urdf.xacro`文件中：

```xml
<robot name="rm_gimbal" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="xyz" default="0.10 0 0.05" />
  <xacro:arg name="rpy" default="0 0 0" />

  <!-- 根坐标系 -->
  <link name="odom" />

  <!-- 云台坐标系 -->
  <link name="gimbal_link" />
  <joint name="gimbal_joint" type="floating">
    <parent link="odom" />
    <child link="gimbal_link" />
  </joint>

  <!-- 相机物理坐标系 -->
  <link name="camera_link" />
  <joint name="camera_joint" type="fixed">
    <origin xyz="$(arg xyz)" rpy="$(arg rpy)" />
    <parent link="gimbal_link" />
    <child link="camera_link" />
  </joint>

  <!-- 相机光学坐标系 -->
  <link name="camera_optical_frame" />
  <joint name="camera_optical_joint" type="fixed">
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}" />
    <parent link="camera_link" />
    <child link="camera_optical_frame" />
  </joint>
</robot>
```

## 常见问题和调试

### 1. TF变换超时
- **现象**：控制台出现"Error while transforming"错误
- **原因**：TF变换链不完整或时间戳不匹配
- **解决方法**：
  - 检查所有TF变换是否正常发布
  - 确认时间戳同步
  - 使用`ros2 run tf2_tools view_frames`生成TF树图

### 2. 坐标系配置错误
- **现象**：目标位置计算错误
- **原因**：相机安装参数配置不正确
- **解决方法**：
  - 检查`odom2camera`参数
  - 重新标定相机安装位置
  - 使用RViz可视化验证坐标系

### 3. 动态变换延迟
- **现象**：云台运动时目标位置滞后
- **原因**：串口通信延迟或TF缓冲区设置
- **解决方法**：
  - 调整`timestamp_offset`参数
  - 增加TF缓冲区大小
  - 检查串口通信频率

## 调试工具

### 1. 可视化TF树
```bash
# 生成TF树图
ros2 run tf2_tools view_frames

# 实时查看TF变换
ros2 run tf2_ros tf2_echo odom gimbal_link
```

### 2. RViz可视化
- 添加TF显示插件
- 设置显示的坐标系和变换关系
- 验证坐标轴方向和原点位置

### 3. 命令行查看
```bash
# 查看当前TF变换
ros2 topic echo /tf

# 查看静态TF变换
ros2 topic echo /tf_static

# 监控TF变换
ros2 run rqt_tf_tree rqt_tf_tree
```

## 设计优势

这个TF树设计确保了：

1. **统一性**：所有传感器数据能正确转换到统一的坐标系
2. **实时性**：云台运动能实时反映在坐标变换中
3. **模块化**：不同模块能在一个统一的坐标系下协同工作
4. **可配置性**：相机安装位置等参数可通过配置文件调整
5. **可扩展性**：可以方便地添加新的传感器或执行器坐标系

## 总结

TF树是RoboMaster视觉系统的核心基础设施，它通过建立清晰的坐标系层次结构，实现了从图像检测到目标跟踪再到弹道计算的完整坐标变换链。理解TF树的结构和工作原理对于系统调试、参数优化和功能扩展都具有重要意义。