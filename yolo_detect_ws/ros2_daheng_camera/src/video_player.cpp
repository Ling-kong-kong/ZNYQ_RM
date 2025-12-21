// Created by Chengfu Zou on 2023.7.2
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

// std
#include <filesystem>
#include <string>
// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// OpenCV
#include <opencv2/opencv.hpp>
// project

namespace fyt::camera_driver {

/**
 * @class VideoPlayerNode
 * @brief 视频播放器节点类
 * @details 用于播放视频文件并发布为ROS2图像话题，支持循环播放和指定起始帧
 */
class VideoPlayerNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   * @details 初始化视频播放器，加载视频文件，设置发布器和定时器
   */
  explicit VideoPlayerNode(const rclcpp::NodeOptions &options)
  : Node("camera_driver", options), frame_cnt_(0) {
    RCLCPP_INFO(this->get_logger(), "Starting VideoPlayerNode!");
    
    // ========== 获取参数 ==========
    video_path = this->declare_parameter("path", "/home/zcf/Downloads/rune.avi");  ///< 视频文件路径
    std::string camera_info_url =
      this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");  ///< 相机标定文件URL
    std::string frame_id = this->declare_parameter("frame_id", "camera_optical_frame");  ///< 相机坐标系ID
    int frame_rate = this->declare_parameter("frame_rate", 30);  ///< 播放帧率
    start_frame_ = this->declare_parameter("start_frame", 0);  ///< 起始帧号
    is_loop_ = this->declare_parameter("keep_looping", true);  ///< 是否循环播放

    
    // ========== 打开视频文件 ==========
    std::filesystem::path video_file(video_path);

    // 检查视频文件是否存在
    if (!std::filesystem::exists(video_file)) {
      RCLCPP_ERROR(this->get_logger(), "Video file %s does not exist!", video_file.string().c_str());
      rclcpp::shutdown();
      return;
    }
    cap_.open(video_path);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Video file open failed!");
      rclcpp::shutdown();
      return;
    }

    // ========== 初始化图像消息 ==========
    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
    image_msg_->header.frame_id = frame_id;
    image_msg_->encoding = sensor_msgs::image_encodings::BGR8;  ///< BGR8编码
    image_msg_->width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);    ///< 从视频获取宽度
    image_msg_->height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);  ///< 从视频获取高度
    image_msg_->step = image_msg_->width * 3;  ///< 每行字节数(宽度*通道数)
    image_msg_->data.resize(image_msg_->step * image_msg_->height);

    // ========== 初始化相机信息 ==========
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "camera_driver", "file://" + video_path);
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_ = camera_info_manager_->getCameraInfo();
    } else {
      // 如果标定文件无效，使用默认参数
      camera_info_manager_->setCameraName(video_path);
      sensor_msgs::msg::CameraInfo camera_info;
      camera_info.header.frame_id = "camera_optical_frame";
      camera_info.header.stamp = this->now();
      camera_info.width = image_msg_->width;
      camera_info.height = image_msg_->height;
      camera_info_manager_->setCameraInfo(camera_info);
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
    camera_info_.header.frame_id = frame_id;
    camera_info_.header.stamp = this->now();

    // ========== 创建相机发布器 ==========
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw");

    // ========== 创建播放循环 ==========
    loop_rate_ = std::make_shared<rclcpp::WallRate>(frame_rate);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      cap_ >> frame_;  ///< 读取一帧
      if (frame_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Video file ends!");
        if (!is_loop_) {
          rclcpp::shutdown();
          return;
        } else {
          // 重新打开视频文件，实现循环播放
          cap_.open(video_path);
          frame_cnt_ = 0;
          return;
        }
      }
      // 将OpenCV Mat数据复制到ROS2图像消息
      memcpy(image_msg_->data.data(), frame_.data, image_msg_->step * image_msg_->height);
      frame_cnt_++;
      
      // 跳过起始帧之前的帧
      if (frame_cnt_ < start_frame_) {
        RCLCPP_INFO(this->get_logger(), "Skipping frame %d", frame_cnt_);
        return;
      }
      
      // 更新时间戳并发布
      image_msg_->header.stamp = camera_info_.header.stamp = this->now();
      camera_pub_.publish(*image_msg_, camera_info_);
      loop_rate_->sleep();  ///< 控制播放帧率
    });

    rclcpp::spin(this->get_node_base_interface());
  }

private:
  std::string video_path;  ///< 视频文件路径
  image_transport::CameraPublisher camera_pub_;  ///< 相机图像发布器
  bool is_loop_;  ///< 是否循环播放标志
  cv::VideoCapture cap_;  ///< OpenCV视频捕获器
  cv::Mat frame_;  ///< 当前帧
  sensor_msgs::msg::Image::SharedPtr image_msg_;  ///< ROS2图像消息
  sensor_msgs::msg::CameraInfo camera_info_;  ///< 相机信息消息
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;  ///< 相机信息管理器
  rclcpp::TimerBase::SharedPtr timer_;  ///< 定时器
  rclcpp::WallRate::SharedPtr loop_rate_;  ///< 墙钟频率控制器
  int start_frame_;  ///< 起始帧号
  int frame_cnt_;  ///< 当前帧计数
};

}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::VideoPlayerNode)
