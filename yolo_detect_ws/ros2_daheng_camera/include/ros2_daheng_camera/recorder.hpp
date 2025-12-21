// Created by Chengfu Zou
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

#ifndef ROS2_DAHENG_CAMERA_RECORDER_HPP_
#define ROS2_DAHENG_CAMERA_RECORDER_HPP_

// std
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>
// OpenCV
#include <opencv2/videoio.hpp>

namespace fyt::camera_driver {

/**
 * @class Recorder
 * @brief 视频录制器类
 * @details 基于OpenCV的多线程视频录制器，支持实时录制相机图像流为视频文件
 */
class Recorder {
public:
  using Frame = std::vector<unsigned char>;  ///< 帧数据类型定义
  
  /**
   * @brief 构造函数
   * @param file 输出视频文件路径
   * @param fps 视频帧率
   * @param size 视频分辨率
   */
  Recorder(const std::filesystem::path &file, int fps, cv::Size size);
  
  /**
   * @brief 析构函数，确保录制线程正确停止
   */
  ~Recorder();

  /**
   * @brief 添加一帧到录制队列
   * @param frame 要录制的帧数据
   */
  void addFrame(const Frame &frame);
  
  /**
   * @brief 开始录制
   */
  void start();
  
  /**
   * @brief 停止录制
   */
  void stop();

  std::filesystem::path path;  ///< 录制文件路径

private:
  /**
   * @brief 录制线程函数
   * @details 独立线程中从队列中取出帧并写入视频文件
   */
  void recorderThread();

  // ========== 视频参数 ==========
  cv::Size size_;               ///< 视频分辨率
  int fps_;                     ///< 视频帧率
  cv::VideoWriter writer_;      ///< OpenCV视频写入器

  // ========== 帧队列管理 ==========
  std::deque<Frame> frame_queue_;  ///< 帧数据队列

  // ========== 线程同步 ==========
  std::mutex mutex_;               ///< 互斥锁，保护帧队列
  std::atomic<bool> recoring_;     ///< 录制状态标志
  std::condition_variable cv_;     ///< 条件变量，用于线程同步
  std::thread recorder_thread_;    ///< 录制线程
};

}  // namespace fyt::camera_driver
#endif  // ROS2_DAHENG_CAMERA_RECORDER_HPP_
