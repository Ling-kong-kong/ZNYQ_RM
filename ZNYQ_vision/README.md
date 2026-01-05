# ZNYQ_vision 项目说明

### 主要特点

- **装甲板自动识别**：使用传统图像处理和深度学习两种方法进行装甲板检测
- **目标跟踪**：基于扩展卡尔曼滤波器的目标状态估计和跟踪
- **弹道解算**：支持重力和空气阻力两种弹道模型
- **相机集成**：支持海康威视(HikVision)和迈德威视(MindVision)工业相机
- **串口通信**：与嵌入式系统进行可靠的数据交换

## 项目结构

```
ZNYQ_vision/
├── rm_auto_aim/             # 自动瞄准模块
│   ├── armor_detector/      # 装甲板检测
│   ├── armor_tracker/       # 装甲板跟踪
│   ├── auto_aim_interfaces/ # 接口定义
│   └── projectile_motion/   # 弹道解算
├── rm_vision/               # 视觉系统集成
│   └── rm_vision_bringup/   # 启动配置
├── rm_serial_driver/        # 串口驱动
├── rm_gimbal_description/   # 云台描述文件
├── rmoss_core/              # RoboMaster OSS 核心模块
├── rmoss_interfaces/        # 通用接口定义
└── ros2_hik_camera/         # 海康相机驱动
```

## 核心模块详解

### 1. armor_detector (装甲板检测模块)

**功能**：负责从相机图像中检测装甲板并解算其三维位置

**核心算法流程**：

传统视觉：

1. **图像预处理**：RGB转灰度图，二值化处理
2. **灯条识别**：轮廓检测，几何筛选，颜色判断
3. **灯条匹配**：将符合条件的灯条配对组合成装甲板
4. **数字识别**：使用ONNX模型识别装甲板上的数字
5. **PnP解算**：计算装甲板在相机坐标系下的三维位姿

神经网络：

1. **图像预处理**：ROI分割，加快处理时间
2. **装甲板识别**：YOLO神经网络识别
3. **PnP解算**：计算装甲板在相机坐标系下的三维位姿

### 2. armor_tracker (装甲板跟踪模块)

**功能**：对检测到的装甲板进行稳定跟踪和状态预测

**核心功能**：

- 坐标系转换（相机坐标系→惯性坐标系）
- 扩展卡尔曼滤波器(EKF)状态估计
- 多目标跟踪和切换
- 目标丢失和重捕获处理

### 3. projectile_motion (弹道解算模块)

**功能**：根据目标位置计算云台控制角度

**支持的弹道模型**：

- **GravityProjectileSolver**：仅考虑重力的抛物线模型
- **GafProjectileSolver**：考虑重力和空气阻力的模型

### 4. 串口通信模块

**功能**：与嵌入式系统进行数据交换

**通信协议**：

- 设备：`/dev/ttyACM0`
- 波特率：115200
- 流控制：无
- 校验：无
- 停止位：1

## 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS版本**：ROS2 Humble
- **编译器**：支持C++14
- **依赖库**：
  - OpenCV
  - OpenVINO (用于深度学习推理)
  - Eigen3
  - yaml-cpp

## 构建与运行

### 1. 环境准备

```bash
# 安装ROS2 Humble
# 参考：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 创建工作空间
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

(可选）神经网络所需环境：

安装依赖项：

- [MindVision SDK](https://mindvision.com.cn/category/software/sdk-installation-package/)或[HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)
- [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)
- [Ceres](http://ceres-solver.org/installation.html)
- 其余：

```
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```

****

***PS：本功能包基于[同济大学 SuperPower 战队 25 赛季自瞄算法开源](https://github.com/TongjiSuperPower/sp_vision_25.git)中/tasks/auto_aim 框架所改 仅移植 yolov5 及其 armor 部分*** 

***PS: 上述为同济官网所需的环境，但理论上仅需安装 OpenVINO 和 相机 SDK 即可***

### 2. 获取源码

```bash
# 克隆项目
git clone https://github.com/Ling-kong-kong/ZNYQ_RM.git

# 安装依赖
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译项目

```bash
colcon build --symlink-install --packages-up-to rm_vision_bringup
```

### 4. 运行系统

```bash
# 设置环境
source install/setup.bash

# 启动完整视觉系统
# 传统视觉
ros2 launch rm_vision_bringup vision_bringup.launch.py
# 神经网络
ros2 launch rm_vision_bringup yolo_vision_bringup.launch.py

# 或启动无硬件版本（用于测试）
ros2 launch rm_vision_bringup no_hardware.launch.py
```



## 配置说明

### 相机配置

文件位置：`rm_vision_bringup/config/node_params.yaml`

```yaml
/camera_node:
  ros__parameters:
    camera_info_url: package://rm_vision_bringup/config/camera_info.yaml
    exposure_time: 2500  # 曝光时间
    gain: 8.0           # 增益
```

### 检测参数配置

```yaml
/armor_detector:
  ros__parameters:
    debug: true
    detect_color: 0     # 0=红色，1=蓝色
    binary_thres: 80    # 二值化阈值
    classifier_threshold: 0.8  # 分类器阈值
```

### 跟踪参数配置

```yaml
/armor_tracker:
  ros__parameters:
    target_frame: odom
    max_armor_distance: 10.0
    ekf:
      sigma2_q_xyz: 0.05
      sigma2_q_yaw: 5.0
      sigma2_q_r: 80.0
```

## 故障排除

### 常见问题

1. **相机无法打开**
   - 确认相机驱动sdk是否正确安装

2. **OpenVINO相关错误**
   - 检查OpenVINO安装路径：`/opt/intel/openvino_2024.6.0/runtime`
   - 设置环境变量：`source /opt/intel/openvino/bin/setupvars.sh`

3. **串口通信失败**
   - 检查串口设备：`ls -l /dev/ttyACM*`
   - 确认设备权限：`sudo chmod 666 /dev/ttyACM0`

4. **编译错误**
   - 清理并重新编译：`colcon build --symlink-install --packages-select [包名]`
   - 检查依赖是否完整：`rosdep check --from-paths src`

### 调试技巧

1. **启用调试输出**

   ```yaml
   /armor_detector:
     ros__parameters:
       debug: true
   ```

2. **查看日志级别**

   ```bash
   ros2 launch rm_vision_bringup vision_bringup.launch.py \
     --ros-args --log-level armor_detector:=DEBUG
   ```

3. **可视化调试**

   - 使用Foxglove Studio进行ROS2数据可视化
   - 查看调试话题：`/debug_armors`, `/debug_lights`
