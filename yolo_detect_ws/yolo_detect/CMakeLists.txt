cmake_minimum_required(VERSION 3.16)

# 添加项目名称
project(yolo_detect)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 查找其他依赖包
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

# 设置OpenVINO路径（根据实际安装路径调整）
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# 查找其他依赖
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# 创建yolo_detect库
add_library(yolo_detect_lib
  src/armor.cpp
  src/yolov5.cpp
)

target_include_directories(yolo_detect_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(yolo_detect_lib
  ${OpenCV_LIBRARIES}
  ${OpenVINO_RUNTIME_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
  "/opt/intel/openvino_2024.6.0/runtime/lib/intel64/libopenvino.so"
)

# 设置OpenVINO包含路径
target_include_directories(yolo_detect_lib PRIVATE "/opt/intel/openvino_2024.6.0/runtime/include")

# 创建可执行文件
add_executable(yolo_detect_node src/yolo_detect_node.cpp)
target_link_libraries(yolo_detect_node yolo_detect_lib)
ament_target_dependencies(yolo_detect_node rclcpp sensor_msgs cv_bridge)
target_include_directories(yolo_detect_node PRIVATE "/opt/intel/openvino_2024.6.0/runtime/include")

# 创建测试可执行文件（如果存在test_yolo_detect.cpp）
# add_executable(yolo_detect_test test_yolo_detect.cpp)
# target_link_libraries(yolo_detect_test yolo_detect_lib)

# 安装规则
install(TARGETS
  yolo_detect_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  yolo_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    models
    DESTINATION share/${PROJECT_NAME}
)

ament_package()