# ROS2参数配置机制说明

## 概述

本文档详细说明ROS2中参数配置文件与节点的关系、调用机制以及使用方法。

## 1. 配置文件结构关系

### 1.1 基本结构

```yaml
# node_params.yaml
/节点名称:
  ros__parameters:
    参数名: 参数值
    嵌套参数:
      子参数: 值
```

### 1.2 实际配置示例

```yaml
/camera_node:
  ros__parameters:
    camera_info_url: package://rm_vision_bringup/config/camera_info.yaml
    exposure_time: 2500
    gain: 8.0

/serial_driver:
  ros__parameters:
    timestamp_offset: 0.006
    device_name: /dev/ttyACM0
    baud_rate: 115200
    flow_control: none
    parity: none
    stop_bits: "1"

/armor_detector:
  ros__parameters:
    debug: true
    detect_color: 0
    binary_thres: 80
    light.min_ratio: 0.1
    armor.min_light_ratio: 0.8
    classifier_threshold: 0.8
    ignore_classes: ["negative"]

/armor_tracker:
  ros__parameters:
    target_frame: odom
    max_armor_distance: 10.0
    ekf:
      sigma2_q_xyz: 0.05
      sigma2_q_yaw: 5.0
      sigma2_q_r: 80.0
      r_xyz_factor: 4e-4
      r_yaw: 5e-3
    tracker:
      max_match_distance: 0.5
      max_match_yaw_diff: 1.0
      tracking_thres: 5
      lost_time_thres: 1.0

/projectile_motion:
  ros__parameters:
    projectile:
      offset_pitch: 0.0
      offset_yaw: 0.0
      offset_time: 0.0
      initial_speed: 18.0
      target_topic: tracker/target
      gimbal_cmd_topic: gimbal_cmd
      shoot_cmd_topic: cmd_shoot
      target_frame: shooter_link
      solver_type: "gravity"
      friction: 0.001
```

## 2. 调用流程详解

### 2.1 Launch文件加载配置

```python
# common.py
import os
from ament_index_python.packages import get_package_share_directory

# 加载配置文件路径
node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 
    'config', 
    'node_params.yaml'
)
```

### 2.2 传递参数给节点

```python
# 组件节点
ComposableNode(
    package='armor_detector',
    plugin='armor_detector::yolo_detect::YoloDetect',
    name='yolo_detector',
    parameters=[node_params],  # 传递整个配置文件
    extra_arguments=[{'use_intra_process_comms': True}]
)

# 独立节点
Node(
    package='armor_tracker',
    executable='armor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],  # 传递配置文件
    ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
)
```

### 2.3 节点声明和读取参数

```cpp
// 基本参数声明
debug_ = this->declare_parameter("debug", false);

// 带描述的参数声明
rcl_interfaces::msg::ParameterDescriptor param_desc;
param_desc.description = "二值化阈值";
int binary_thres = declare_parameter("binary_thres", 160, param_desc);

// 嵌套参数声明
double initial_speed = this->declare_parameter("projectile.initial_speed", 18.0);

// 数组参数声明
this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
```

## 3. 参数映射机制

### 3.1 节点名称匹配

| 配置文件路径 | 节点名称 | 参数访问方式 |
|-------------|----------|-------------|
| `/camera_node/exposure_time` | `camera_node` | `declare_parameter("exposure_time")` |
| `/armor_detector/debug` | `armor_detector` | `declare_parameter("debug")` |
| `/projectile_motion/projectile/initial_speed` | `projectile_motion` | `declare_parameter("projectile.initial_speed")` |

### 3.2 嵌套参数访问

```yaml
# 配置文件
/armor_tracker:
  ros__parameters:
    ekf:
      sigma2_q_xyz: 0.05
      sigma2_q_yaw: 5.0
    tracker:
      max_match_distance: 0.5
```

```cpp
// 节点中访问
double sigma_xyz = this->declare_parameter("ekf.sigma2_q_xyz", 0.05);
double sigma_yaw = this->declare_parameter("ekf.sigma2_q_yaw", 5.0);
double match_distance = this->declare_parameter("tracker.max_match_distance", 0.5);
```

## 4. 参数覆盖机制

### 4.1 命令行覆盖

```bash
# 覆盖单个参数
ros2 launch rm_vision_bringup yolo_vision_bringup.launch.py \
  armor_detector.debug:=false

# 覆盖嵌套参数
ros2 launch rm_vision_bringup yolo_vision_bringup.launch.py \
  projectile_motion.projectile.initial_speed:=20.0

# 覆盖数组参数
ros2 launch rm_vision_bringup yolo_vision_bringup.launch.py \
  armor_detector.ignore_classes:='["negative", "guard"]'
```

### 4.2 Launch文件中覆盖

```python
# 方法1：直接指定参数
parameters=[
    node_params,
    {"debug": false}  # 覆盖配置文件中的值
]

# 方法2：使用参数字典
custom_params = {
    "debug": False,
    "binary_thres": 90
}
parameters=[custom_params]

# 方法3：合并参数
import yaml
with open(node_params, 'r') as f:
    config = yaml.safe_load(f)

config['/armor_detector']['ros__parameters']['debug'] = False
parameters=[config]
```

## 5. 动态参数更新

### 5.1 设置参数回调

```cpp
// 构造函数中设置回调
parameter_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters) {
        return parameter_callback(parameters);
    });

// 回调函数实现
rcl_interfaces::msg::SetParametersResult
ArmorDetectorNode::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
        if (param.get_name() == "debug") {
            debug_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Debug模式更新为: %s", 
                       debug_ ? "开启" : "关闭");
        }
        // 处理其他参数...
    }
    
    return result;
}
```

### 5.2 获取和设置参数

```cpp
// 获取参数
auto debug_param = this->get_parameter("debug");
bool debug_value = debug_param.as_bool();

// 设置参数
this->set_parameter(rclcpp::Parameter("debug", true));

// 获取所有参数
auto parameters = this->get_parameters({"debug", "binary_thres"});
```

## 6. 配置文件路径解析

### 6.1 路径类型

```yaml
# 包内资源路径
camera_info_url: package://rm_vision_bringup/config/camera_info.yaml

# 相对路径
model_path: models/yolo_detect/model.onnx

# 绝对路径
log_path: /tmp/ros_logs/
```

### 6.2 路径解析代码

```cpp
// 解析package://路径
std::string config_path = this->get_parameter("config_path").as_string();
if (config_path.find("package://") == 0) {
    // 替换package://为实际包路径
    config_path.replace(0, 10, 
        ament_index_cpp::get_package_share_directory("package_name"));
}
```

## 7. 参数类型支持

### 7.1 基本类型

```cpp
// 布尔型
bool debug = this->declare_parameter("debug", false);

// 整型
int threshold = this->declare_parameter("threshold", 80);

// 浮点型
double ratio = this->declare_parameter("ratio", 0.5);

// 字符串
std::string topic = this->declare_parameter("topic", "/image_raw");
```

### 7.2 复合类型

```cpp
// 数组
std::vector<double> matrix = this->declare_parameter("matrix", 
    std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

// 字符串数组
std::vector<std::string> classes = this->declare_parameter("classes", 
    std::vector<std::string>{"class1", "class2"});
```

## 8. 最佳实践

### 8.1 参数命名规范

- 使用小写字母和下划线
- 嵌套参数使用点号分隔
- 名称要有描述性

```yaml
# 好的命名
camera_exposure_time: 2500
detector.confidence_threshold: 0.7

# 避免的命名
exposure: 2500
threshold: 0.7
```

### 8.2 默认值设置

```cpp
// 总是提供合理的默认值
bool debug = this->declare_parameter("debug", false);  // 默认关闭debug
double threshold = this->declare_parameter("threshold", 0.5);  // 默认中等阈值
```

### 8.3 参数验证

```cpp
// 在参数回调中验证参数值
if (param.get_name() == "threshold") {
    double value = param.as_double();
    if (value < 0.0 || value > 1.0) {
        result.successful = false;
        result.reason = "阈值必须在0.0到1.0之间";
    }
}
```

## 9. 调试技巧

### 9.1 查看参数值

```bash
# 查看节点所有参数
ros2 param list /node_name

# 查看特定参数值
ros2 param get /node_name parameter_name

# 查看参数描述
ros2 param describe /node_name parameter_name
```

### 9.2 动态修改参数

```bash
# 设置参数值
ros2 param set /node_name parameter_name value

# 保存参数到文件
ros2 param dump /node_name > params.yaml
```

## 10. 常见问题和解决方案

### 10.1 参数未找到错误

**问题**: `Parameter 'xxx' not declared`

**解决**: 确保在节点构造函数中使用`declare_parameter()`声明参数

### 10.2 参数类型不匹配

**问题**: `Parameter type mismatch`

**解决**: 检查配置文件中的参数类型与节点期望的类型一致

### 10.3 嵌套参数访问失败

**问题**: 无法访问嵌套参数

**解决**: 使用点号分隔的完整路径，如`"projectile.initial_speed"`

### 10.4 配置文件路径错误

**问题**: `Failed to load parameter file`

**解决**: 检查文件路径是否正确，使用`package://`前缀引用包内资源

---

本文档涵盖了ROS2参数配置的主要方面，为开发者提供了完整的参数管理指南。