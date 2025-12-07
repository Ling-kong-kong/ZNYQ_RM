# Gemini 代码助手上下文

本文档为 Gemini 代码助手提供了上下文，以帮助其理解此工作区的结构和目的。

## 项目概览

此工作区包含一个用于 RoboMaster（一项年度机器人竞赛）的 ROS 2 项目。该项目以计算机视觉任务为中心，特别是自动瞄准。它似乎由两大部分组成：`rm_vision` 和 `rmoss_core`。

### rm_vision

`rm_vision` 是一个完整的自动瞄准解决方案。它包括以下包：

*   **`rm_auto_aim`**: 核心自动瞄准逻辑，包括装甲板检测和跟踪。
*   **`ros2_hik_camera`**: 海康威视工业相机的驱动程序。
*   **`rm_gimbal_description`**: 描述机器人云台的 URDF 文件。
*   **`rm_serial_driver`**: 用于与机器人硬件进行串行通信的驱动程序。

### rmoss_core

`rmoss_core` 是 RoboMaster 开发的基础包集合。它提供了可重用的组件，包括：

*   **`rmoss_util`**: 用于调试、图像处理和 ROS 2 的实用函数。
*   **`rmoss_interfaces`**: ROS 2 消息和服务定义。

*   **`rmoss_base`**: 用于与嵌入式系统接口的通信工具。
*   **`rmoss_cam`**: 相机驱动程序和工具。
*   **`rmoss_projectile_motion`**: 用于求解弹道运动方程的库。

## 构建和运行

这是一个 ROS 2 项目，应该使用 `colcon` 进行构建。

### 构建项目

要构建项目，请在工作区根目录中运行以下命令：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 运行项目

`rm_vision` 项目可以使用以下命令启动：

```bash
bash ros2 launch rm_vision_bringup vision_bringup.launch.py
```

`rmoss_core` 包可能旨在用作库，并且可能没有自己的启动文件。

## 开发约定

该项目似乎遵循标准的 ROS 2 开发约定。

*   **代码风格**: 项目使用 `.clang-format` 和 `.clang-tidy` 文件来强制执行一致的代码风格。
*   **测试**: 项目使用 `colcon test` 进行测试。
*   **依赖项**: 依赖项使用 `rosdep` 进行管理。