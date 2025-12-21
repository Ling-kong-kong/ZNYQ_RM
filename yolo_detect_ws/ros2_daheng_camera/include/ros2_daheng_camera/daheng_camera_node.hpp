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

#ifndef ROS2_DAHENG_CAMERA_DAHENG_CAMERA_NODE_HPP_
#define ROS2_DAHENG_CAMERA_DAHENG_CAMERA_NODE_HPP_

// std
#include <memory>
// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// Daheng Galaxy Driver
#include "daheng/DxImageProc.h"
#include "daheng/GxIAPI.h"
// project
#include "ros2_daheng_camera/recorder.hpp"


namespace fyt::camera_driver {

/**
 * @class DahengCameraNode
 * @brief 大恒工业相机驱动节点类
 * @details 基于ROS2的大恒Galaxy系列工业相机驱动，支持图像采集、参数配置、录制等功能
 */
class DahengCameraNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   */
  explicit DahengCameraNode(const rclcpp::NodeOptions &options);

  /**
   * @brief 析构函数
   */
  ~DahengCameraNode();

  /**
   * @brief 打开相机设备
   * @return 打开成功返回true，失败返回false
   */
  bool open();
  
  /**
   * @brief 关闭相机设备
   */
  void close();

  /**
   * @brief 获取最新帧的时间戳
   * @return 最新帧的时间戳
   */
  rclcpp::Time getLatestFrameStamp();

private:
  // ========== 看门狗相关 ==========
  /**
   * @brief 定时器回调函数，用于监控相机状态
   */
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器指针

  

  // ========== 参数回调相关 ==========
  /**
   * @brief 参数设置回调函数
   * @param parameters 参数列表
   * @return 参数设置结果
   */
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    std::vector<rclcpp::Parameter> parameters);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;  ///< 参数回调句柄

  // ========== 图像回调相关 ==========
  /**
   * @brief 大恒相机帧回调函数
   * @param pFrame 帧参数指针
   */
  void GX_STDC onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame);

  // ========== ROS2发布器相关 ==========
  image_transport::CameraPublisher pub_;  ///< 相机图像发布器
  sensor_msgs::msg::Image image_msg_;     ///< 图像消息
  sensor_msgs::msg::CameraInfo camera_info_;  ///< 相机信息消息
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;  ///< 相机信息管理器
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_;  ///< 相机配置参数

  // ========== 大恒Galaxy API相关 ==========
  GX_DEV_HANDLE dev_handle_;           ///< 大恒设备句柄
  GX_STATUS gx_status_;               ///< 大恒API状态
  int64_t gx_pixel_format_;           ///< 像素格式
  int64_t gx_pay_load_size_;          ///< 载荷大小
  int64_t gx_bayer_type_;             ///< Bayer格式类型
  int64_t max_resolution_width_;      ///< 最大分辨率宽度
  int64_t max_resolution_height_;     ///< 最大分辨率高度

  // ========== 录制器相关 ==========
  std::unique_ptr<Recorder> recorder_;  ///< 录制器指针

  // ========== 通用参数 ==========
  bool is_open_ = false;           ///< 相机是否打开标志
  int resolution_width_;           ///< 分辨率宽度
  int resolution_height_;          ///< 分辨率高度
  int auto_white_balance_;         ///< 自动白平衡设置
  int frame_rate_;                 ///< 帧率设置
  int exposure_time_;              ///< 曝光时间设置
  double gain_;                    ///< 增益设置
  int offest_x_;                   ///< X轴偏移
  int offset_y_;                   ///< Y轴偏移
};

}  // namespace fyt::camera_driver
#endif  // ROS2_DAHENG_CAMERA_DAHENG_CAMERA_NODE_HPP_
