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

# 检查OpenVINO是否可用
set(OpenVINO_RUNTIME_ROOT "/opt/intel/openvino_2024.6.0/runtime")
set(OpenVINO_LIB_PATH "${OpenVINO_RUNTIME_ROOT}/lib/intel64")
set(OpenVINO_INCLUDE_PATH "${OpenVINO_RUNTIME_ROOT}/include")
set(OpenVINO_LIBRARIES "${OpenVINO_LIB_PATH}/libopenvino.so")
set(OpenVINO_INCLUDE_DIRS "${OpenVINO_INCLUDE_PATH}")

# 检查OpenVINO库文件是否存在
if(EXISTS "${OpenVINO_LIBRARIES}" AND EXISTS "${OpenVINO_INCLUDE_PATH}")
  set(OPENVINO_FOUND TRUE)
  message(STATUS "OpenVINO found at: ${OpenVINO_RUNTIME_ROOT}")
else()
  set(OPENVINO_FOUND FALSE)
  message(WARNING "OpenVINO NOT found at expected location: ${OpenVINO_RUNTIME_ROOT}")
  message(WARNING "YOLO detection node will NOT be built!")
  message(WARNING "To enable YOLO detection, please install OpenVINO or update the path in CMakeLists.txt")
endif()

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
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 条件性添加OpenVINO相关配置
if(OPENVINO_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenVINO_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OpenVINO_LIBRARIES})
  # 确保OpenVINO运行时库路径正确
  target_link_directories(${PROJECT_NAME} PUBLIC ${OpenVINO_LIB_PATH})
  # 添加编译定义，启用YOLO功能
  target_compile_definitions(${PROJECT_NAME} PUBLIC OPENVINO_AVAILABLE)
endif()

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  yaml-cpp
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN armor_detector::tradition_detect::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

# 条件性注册YOLO检测节点
if(OPENVINO_FOUND)
  # 注册YOLO检测节点
  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN armor_detector::yolo_detect::YoloDetect
    EXECUTABLE yolo_detect_node
  )
else()
  message(STATUS "Skipping YOLO detection node registration due to missing OpenVINO")
endif()

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

# 安装基本配置文件
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 条件性安装YOLO相关文件
if(OPENVINO_FOUND)
  install(DIRECTORY
    models
    DESTINATION share/${PROJECT_NAME}/
  )
  message(STATUS "YOLO models will be installed")
else()
  message(STATUS "Skipping YOLO models installation due to missing OpenVINO")
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)
