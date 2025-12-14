# Project Overview

This ROS 2 workspace contains two primary packages: `ros2_hik_camera` and `yolo_detect`.

## `ros2_hik_camera`

This package provides an interface for Hikvision USB3.0 industrial cameras within the ROS 2 ecosystem. It is responsible for:
- Enumerating and connecting to Hikvision USB3.0 cameras.
- Grabbing image frames from the camera.
- Converting image pixel types (e.g., to `rgb8`).
- Publishing `sensor_msgs/Image` (raw camera images) and `sensor_msgs/CameraInfo` topics.
- Exposing camera parameters like `exposure_time` and `gain` as ROS parameters for dynamic configuration.

**Key Technologies:**
- ROS 2 (C++)
- Hikvision SDK
- `image_transport`, `camera_info_manager`, `cv_bridge`

## `yolo_detect`

This package implements YOLOv5 object detection, specifically for RoboMaster armor plates, leveraging OpenVINO for optimized inference. Its main functionalities include:
- Subscribing to an image topic (expected to be from a camera, e.g., `/image_raw` from `ros2_hik_camera`).
- Performing YOLOv5 detection on incoming image frames using an OpenVINO inference engine.
- Supporting Region of Interest (ROI) for detection, configurable via ROS parameters and a YAML configuration file (`config/sentry.yaml`).
- Calculating and displaying frames per second (FPS) for detection.
- Optionally publishing a visualized image with bounding boxes and detection information for debugging.
- Converting detected armor plates into a custom `rm_auto_aim::Armor` message format.

**Key Technologies:**
- ROS 2 (C++)
- YOLOv5
- OpenVINO (explicitly configured for `/opt/intel/openvino_2024.6.0`)
- OpenCV
- `yaml-cpp`, `Eigen`

# Building and Running

This project uses `colcon` as its build system for ROS 2 packages.

## Build

To build the entire workspace, navigate to the root directory (`yolo_detect_ws/`) and run:
```bash
colcon build
```

## Source

After building, you need to source the workspace to make the ROS 2 packages available in your environment. Given the Windows operating system, use:
```bash
install/setup.bat
```
(On Linux, this would typically be `source install/setup.bash`)

## Running the Nodes

### Launching the Hikvision Camera Node

To start the camera node:
```bash
ros2 launch hik_camera hik_camera.launch.py
```
This launch file uses parameters from `config/camera_params.yaml` and `config/camera_info.yaml`. You can override `use_sensor_data_qos`.

### Launching the YOLO Detection Node

To start the YOLO detection node:
```bash
ros2 launch yolo_detect yolo_detect.launch.py
```
This launch file allows configuring:
- `config_file`: Path to the YAML configuration for the detector (default: `config/sentry.yaml`).
- `debug_mode`: Boolean to enable/disable debug visualization (default: `true`).
- `image_topic`: The input image topic to subscribe to (default: `/image_raw`).

# Development Conventions

## Language Standards

- `ros2_hik_camera`: C++14
- `yolo_detect`: C++17

## Linting and Testing

Both packages utilize `ament_lint_auto` and `ament_lint_common` for code quality checks.
The `ros2_hik_camera` package explicitly excludes `ament_cmake_copyright`, `ament_cmake_cpplint`, and `ament_cmake_uncrustify`, which may indicate specific in-house linting preferences or reliance on other formatting tools.

## OpenVINO Configuration

The `yolo_detect` package has a hardcoded dependency on the OpenVINO installation path: `/opt/intel/openvino_2024.6.0/runtime/cmake`. Ensure OpenVINO is installed at this location or modify `yolo_detect/CMakeLists.txt` accordingly.
