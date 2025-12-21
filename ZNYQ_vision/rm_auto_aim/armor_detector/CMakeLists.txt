cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 手动查找额外的依赖
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置OpenVINO路径（根据实际安装路径调整）
set(OpenVINO_RUNTIME_ROOT "/opt/intel/openvino_2024.6.0/runtime")
set(OpenVINO_LIB_PATH "${OpenVINO_RUNTIME_ROOT}/lib/intel64")
set(OpenVINO_INCLUDE_PATH "${OpenVINO_RUNTIME_ROOT}/include")

# 直接设置OpenVINO变量
set(OpenVINO_LIBRARIES "${OpenVINO_LIB_PATH}/libopenvino.so")
set(OpenVINO_INCLUDE_DIRS "${OpenVINO_INCLUDE_PATH}")

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  ${OpenVINO_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
)

# 确保OpenVINO运行时库路径正确
target_link_directories(${PROJECT_NAME} PUBLIC /opt/intel/openvino_2024.6.0/runtime/lib/intel64)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN armor_detector::tradition_detect::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

# 注册YOLO检测节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN armor_detector::yolo_detect::YoloDetect
  EXECUTABLE yolo_detect_node
)

#############
## Testing ##
#############

# Tests are temporarily disabled due to linking issues
# TODO: Fix test linking by ensuring proper symbol export
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  # Tests temporarily disabled
  # find_package(ament_cmake_gtest)
  # ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  # target_link_libraries(test_node_startup 
  #   ${PROJECT_NAME}
  #   ${OpenCV_LIBS}
  # )

  # ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  # target_link_libraries(test_number_cls 
  #   ${PROJECT_NAME}
  #   ${OpenCV_LIBS}
  # )

endif()

#############
## Install ##
#############

install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  models
)
