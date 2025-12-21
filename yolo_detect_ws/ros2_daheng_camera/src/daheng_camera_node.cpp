// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_daheng_camera/daheng_camera_node.hpp"
// std
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <thread>
// ros2
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
// Daheng Galaxy Driver
#include "daheng/GxIAPI.h"
// project

// ========== 大恒API回调适配器 ==========
/**
 * @brief 全局回调函数包装器
 * @details 用于适配大恒Galaxy API的C风格回调接口，转换为C++的std::function
 */
static std::function<void GX_STDC(GX_FRAME_CALLBACK_PARAM *)> g_callback;

/**
 * @brief C风格回调包装函数
 * @param arg 帧回调参数指针
 * @details 作为大恒API的回调入口，内部调用C++函数对象
 */
extern "C" void GX_STDC CallbackWrapper(GX_FRAME_CALLBACK_PARAM *arg)
{
  if (g_callback)
  {
    g_callback(arg);
  }
}

namespace fyt::camera_driver
{

  /**
   * @brief 大恒相机节点构造函数
   * @param options ROS2节点选项
   * @details 初始化相机驱动，设置参数，创建发布器和订阅器
   */
  DahengCameraNode::DahengCameraNode(const rclcpp::NodeOptions &options)
  : Node("camera_driver", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DahengCameraNode!");

    // ========== 声明并获取相机参数 ==========
    camera_name_ = this->declare_parameter("camera_name", "daheng"); ///< 相机设备名称
    camera_info_url_ =
        this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml"); ///< 相机标定文件路径
    frame_id_ = this->declare_parameter("camera_frame_id", "camera_optical_frame");                 ///< 相机坐标系ID
    pixel_format_ = this->declare_parameter("pixel_format", "rgb8");                                ///< 像素格式
    resolution_width_ = this->declare_parameter("resolution_width", 1280);                          ///< 图像宽度
    resolution_height_ = this->declare_parameter("resolution_height", 1024);                        ///< 图像高度
    auto_white_balance_ = this->declare_parameter("auto_white_balance", 1);                         ///< 自动白平衡开关
    frame_rate_ = this->declare_parameter("frame_rate", 210);                                       ///< 帧率设置
    exposure_time_ = this->declare_parameter("exposure_time", 2000);                                ///< 曝光时间(微秒)
    gain_ = this->declare_parameter("gain", 5.0);                                                   ///< 增益设置
    offest_x_ = this->declare_parameter("offsetX", 0);                                              ///< X轴偏移
    offset_y_ = this->declare_parameter("offsetY", 0);                                              ///< Y轴偏移

    // ========== 像素格式转换 ==========
      // 将ROS2像素格式参数转换为大恒API对应的像素格式常量
      if (pixel_format_ == "mono8") {
        gx_pixel_format_ = GX_PIXEL_FORMAT_MONO8;  ///< 8位单色
      } else if (pixel_format_ == "mono16") {
        gx_pixel_format_ = GX_PIXEL_FORMAT_MONO16;  ///< 16位单色
      } else if (pixel_format_ == "bgr8") {
        gx_pixel_format_ = GX_PIXEL_FORMAT_BGR8;    ///< 8位BGR彩色
        gx_bayer_type_ = BAYERBG;                   ///< BG Bayer格式
      } else if (pixel_format_ == "rgb8") {
        gx_pixel_format_ = GX_PIXEL_FORMAT_RGB8;    ///< 8位RGB彩色
        gx_bayer_type_ = BAYERRG;                   ///< RG Bayer格式
      } else if (pixel_format_ == "bgra8") {
        gx_pixel_format_ = GX_PIXEL_FORMAT_BGRA8;   ///< 8位BGRA彩色
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: %s", pixel_format_.c_str());
      }
    
      // ========== 相机信息管理器初始化 ==========
  // ========== 相机信息管理器初始化 ==========
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);

  // 加载相机标定信息
  if (camera_info_manager_->validateURL(camera_info_url_))
  {
    camera_info_manager_->loadCameraInfo(camera_info_url_);
    camera_info_ = camera_info_manager_->getCameraInfo();
  }
  else
  {
    // 如果标定文件无效，使用默认参数
    camera_info_manager_->setCameraName(camera_name_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = image_msg_.width;
    camera_info.height = image_msg_.height;
    camera_info_manager_->setCameraInfo(camera_info);
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url_.c_str());
  }
  camera_info_.header.frame_id = frame_id_;
  camera_info_.header.stamp = this->now();

  // ========== 图像发布器初始化 ==========
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  

  // ========== 看门狗定时器初始化 ==========
  // 每秒检查一次相机状态
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                   std::bind(&DahengCameraNode::timerCallback, this));

  // ========== 参数回调初始化 ==========
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DahengCameraNode::onSetParameters, this, std::placeholders::_1));

  // ========== 录制器初始化 ==========
  bool enable_recorder = this->declare_parameter("recording", false);
  if (enable_recorder)
  {
    std::string home = std::getenv("HOME"); // 获取用户主目录

    // 构建视频文件路径：~/fyt2024-log/video/时间戳.avi
    namespace fs = std::filesystem;
    std::filesystem::path video_path = fs::path(home) / "fyt2024-log/video/" /
                                       std::string(std::to_string(std::time(nullptr)) + ".avi");

    // 创建录制器实例
    recorder_ = std::make_unique<Recorder>(
        video_path, frame_rate_, cv::Size(resolution_width_, resolution_height_));
    recorder_->start();
    RCLCPP_INFO(this->get_logger(), "Recorder started! Video file: %s", video_path.string().c_str());
  }

  RCLCPP_INFO(this->get_logger(), "DahengCameraNode has been initialized!");
}

DahengCameraNode::~DahengCameraNode()
{
  close();
  if (recorder_ != nullptr)
  {
    recorder_->stop();
RCLCPP_INFO(
  this->get_logger(),
  "DahengCameraNode has been destroyed!");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "DahengCameraNode has been destroyed!");
}

void DahengCameraNode::timerCallback()
{
  // Open camera
  while (!is_open_ && rclcpp::ok())
  {
    bool is_open_success = open();
    if (!is_open_success)
    {
      RCLCPP_ERROR(this->get_logger(), "open() failed");
      close();
      return;
    }
  }
  // Watch Dog
  double dt = (this->now() - rclcpp::Time(camera_info_.header.stamp)).seconds();
  if (dt > 5.0)
  {
    RCLCPP_WARN(this->get_logger(), "Camera is not alive! lost frame for %.2f seconds", dt);
    close();
  }
}

void DahengCameraNode::close()
{
  RCLCPP_INFO(this->get_logger(), "Closing Daheng Galaxy Camera Device!");
  if (is_open_ && dev_handle_ != nullptr)
  {
    // 发送停采命令
    GXStreamOff(dev_handle_);
    GXCloseDevice(dev_handle_);
    // GXUnregisterCaptureCallback(dev_handle_);
  }
  GXCloseLib();
  is_open_ = false;
}

bool DahengCameraNode::open()
{
  RCLCPP_INFO(this->get_logger(), "Opening Daheng Galaxy Camera Device!");
  if (is_open_)
  {
    RCLCPP_WARN(this->get_logger(), "Daheng Galaxy Camera Device is already opened!");
    close();
  }
  gx_status_ = GX_STATUS_SUCCESS;
  GX_OPEN_PARAM openParam;
  uint32_t device_num = 0;
  openParam.accessMode = GX_ACCESS_EXCLUSIVE;
  openParam.openMode = GX_OPEN_INDEX;
  openParam.pszContent = (char *)"1";
  // 尝试初始化库
  gx_status_ = GXInitLib();
  if (gx_status_ != GX_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Can't init lib");
    return false;
  }

  // 枚举设备列表
  gx_status_ = GXUpdateDeviceList(&device_num, 1000);
  if ((gx_status_ != GX_STATUS_SUCCESS) || (device_num <= 0))
  {
    RCLCPP_WARN(this->get_logger(), "Can't find camera");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Found %d devices", device_num);
  // 打开设备
  gx_status_ = GXOpenDevice(&openParam, &dev_handle_);
  if (gx_status_ != GX_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Can't open device");
    return false;
  }
  is_open_ = true;

  GXGetInt(dev_handle_, GX_INT_WIDTH_MAX, &max_resolution_width_);
  GXGetInt(dev_handle_, GX_INT_HEIGHT_MAX, &max_resolution_height_);

  // 设置像素格式
  GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, gx_pixel_format_);
  // 设置宽度
  GXSetInt(dev_handle_, GX_INT_WIDTH, resolution_width_);
  // 设置高度
  GXSetInt(dev_handle_, GX_INT_HEIGHT, resolution_height_);

  // // 从中心裁剪
  // int64_t offest_x_ = (MAX_RESOLUTION_WIDTH - resolution_width_) / 2;
  // int64_t offset_y_ = (MAX_RESOLUTION_HEIGHT - resolution_height_) / 2;
  //  GXSetEnum(dev_handle_, GX_ENUM_RREGION_SELECTOR,
  //  GX_REGION_SELECTOR_REGION0);
  GXSetInt(dev_handle_, GX_INT_OFFSET_X, offest_x_);
  GXSetInt(dev_handle_, GX_INT_OFFSET_Y, offset_y_);

  // 获取图像大小
  GXGetInt(dev_handle_, GX_INT_PAYLOAD_SIZE, &gx_pay_load_size_);
  // 设置是否开启自动白平衡
  if (auto_white_balance_)
  {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, 1);
  }
  else
  {
    // 设置白平衡数值  如果开启自动白平衡则无效
    GXSetEnum(dev_handle_, GX_ENUM_LIGHT_SOURCE_PRESET, GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K);
  }
  // 设置帧率
  GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ENUM_COVER_FRAMESTORE_MODE_ON);
  GXSetFloat(dev_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate_);

  // 设置曝光时间
  GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  // 设置增益
  GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_);

  // 设置采集模式连续采集
  GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GXSetInt(dev_handle_, GX_INT_ACQUISITION_SPEED_LEVEL, 1);

  // 注册图像处理回调函数
  //  void (*)(GX_FRAME_CALLBACK_PARAM *)
  g_callback = std::bind(&DahengCameraNode::onFrameCallbackFun, this, std::placeholders::_1);
  gx_status_ = GXRegisterCaptureCallback(dev_handle_, nullptr, CallbackWrapper);
  if (gx_status_ != GX_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Register capture callback function failed!");
    return false;
  }

  // 发送开采命令
  gx_status_ = GXStreamOn(dev_handle_);
  if (gx_status_ != GX_STATUS_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Send start capture command failed!");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Daheng Galaxy Camera Device Open Success!");
  return true;
}

void GX_STDC DahengCameraNode::onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame)
{
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS)
  {
    // RGB转换
    DxRaw8toRGB24((void *)pFrame->pImgBuf,
                  image_msg_.data.data(),
                  pFrame->nWidth,
                  pFrame->nHeight,
                  RAW2RGB_NEIGHBOUR,
                  static_cast<DX_PIXEL_COLOR_FILTER>(gx_bayer_type_),
                  false);
    image_msg_.header.stamp = camera_info_.header.stamp = this->now();
    pub_.publish(image_msg_, camera_info_);
    if (recorder_ != nullptr)
    {
      recorder_->addFrame(image_msg_.data);
    }
  }
}

rcl_interfaces::msg::SetParametersResult DahengCameraNode::onSetParameters(
    std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters)
  {
    if (param.get_name() == "exposure_time")
    {
      exposure_time_ = param.as_int();
      GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, static_cast<double>(exposure_time_));
      RCLCPP_INFO(this->get_logger(), "Set exposure_time: %d", exposure_time_);
    }
    else if (param.get_name() == "gain")
    {
      gain_ = param.as_double();
      GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_);
      RCLCPP_INFO(this->get_logger(), "Set gain: %f", gain_);
    }
  }
  return result;
}

} // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::DahengCameraNode)
