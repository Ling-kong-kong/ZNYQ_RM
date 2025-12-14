# yolo_detect 包 - 总体功能和工作流程

## 看前须知

本功能包基于[同济大学 SuperPower 战队 25 赛季自瞄算法开源](https://github.com/TongjiSuperPower/sp_vision_25.git)中/tasks/auto_aim 框架所改 仅移植 yolov5 及其 armor 部分 

通过部分适应性修改完成一下功能：

1. 成功移植 ros2 依赖，订阅海康相机发布的图像信息。

2. 仿照 RV debug 模式实现 /result_img 发布。

3. 支持定义 ROI 以将检测限制在图像的特定区域，ROI 参数可以动态更新。

4. 将内部 `yolo_detect::Armor` 结构转换为 `rm_auto_aim::Armor` 消息。

   注： `rm_auto_aim::Armor` 是 RV 的 armor 的数据类型 原本打算是为了完全复刻 RV 的 debug，后面懒得做了，就做到了这种地步。

   

   PS: 本文档为25.12.11所著

## 所需环境

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

***PS: 上述为同济官网所需的环境，但理论上仅需安装 OpenVINO 和 海康相机 SDK 即可***

## 工作流程/流程图

以下流程图说明了 `yolo_detect` 节点的高级操作流程：

```mermaid
graph TD
    A["YoloDetect 节点初始化"] --> B["声明并加载 ROS 参数"]
    B --> C{"从 config/sentry.yaml 读取配置 (ROI, 图像尺寸)"}
    C --> D{"验证初始 ROI 边界"}
    D -- 有效 --> E["使用 OpenVINO 初始化 YOLOv5 检测器"]
    D -- 无效 --> F["记录错误并退出节点"]

    E --> G["创建图像订阅者: /image_raw"]
    E --> H["创建结果图像发布者: /result_img"]
    E --> I["注册参数回调以进行动态 ROI 更新"]

    subgraph 主要 ROS 循环
        J{"收到新图像? (image_callback)"}
        J -- 是 --> K["将 ROS 图像转换为 OpenCV Mat"]
        K --> L{"图像为空?"}
        L -- 是 --> M["记录警告并跳过帧"]
        L -- 否 --> N["如果启用，应用 ROI 裁剪"]
        N --> O["预处理图像以进行 YOLOv5 输入 (缩放, 填充)"]
        O --> P["执行 OpenVINO 推理"]
        P --> Q["后处理 YOLO 输出: 解析检测, NMS"]
        Q --> R["验证检测到的装甲板 (名称, 类型一致性)"]
        R --> S["将 yolo_detect::Armor 转换为 rm_auto_aim::Armor"]
        S --> T["计算并平滑 FPS"]
        T --> U{"调试模式激活?"}
        U -- 是 --> V["在图像上绘制检测和 ROI"]
        V --> W["将可视化图像发布到 /result_img"]
        U -- 否 --> X["将检测结果和 FPS 记录到控制台"]
        X --> J
        W --> J

        Y{"参数已更改? (roi_parameter_callback)"}
        Y -- 是 --> Z["验证新的 ROI 参数"]
        Z -- 有效 --> AA["更新内部 ROI 和 YOLOv5 检测器的 ROI"]
        Z -- 无效 --> BB["记录错误并拒绝更改"]
        BB --> Y
        AA --> Y
    end
```

详细描述请看[yolo_detect_functions.md](yolo_detect_functions.md) 此文档由LLM大模型所著 注意辨明

