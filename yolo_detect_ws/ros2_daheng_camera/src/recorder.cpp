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

#include "ros2_daheng_camera/recorder.hpp"
// std
#include <filesystem>
// OpenCV
#include <opencv2/opencv.hpp>
// project

namespace fyt::camera_driver {

/**
 * @brief 录制器构造函数
 * @param file 输出视频文件路径
 * @param fps 视频帧率
 * @param size 视频分辨率
 * @details 初始化录制器参数，但不开始录制
 */
Recorder::Recorder(const std::filesystem::path &file, int fps, cv::Size size)
: path(file), size_(size), fps_(fps) {}

/**
 * @brief 开始录制
 * @details 创建输出目录，初始化视频写入器，启动录制线程
 */
void Recorder::start() {
  // 如果输出目录不存在，创建目录
  if (!std::filesystem::exists(path)) {
    std::filesystem::create_directories(path.parent_path());
  }

  // 初始化OpenCV视频写入器，使用MJPEG编码
  writer_.open(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, size_, true);
  recoring_ = true;
  recorder_thread_ = std::thread(&Recorder::recorderThread, this);  // 启动录制线程
}

/**
 * @brief 析构函数
 * @details 确保录制器正确停止
 */
Recorder::~Recorder() { stop(); }

/**
 * @brief 添加一帧到录制队列
 * @param frame 要录制的帧数据
 * @details 线程安全地添加帧到队列，如果队列满了则丢弃新帧
 */
void Recorder::addFrame(const Frame &frame) {
  mutex_.lock();
  // 限制队列大小，避免内存占用过大
  if (frame_queue_.size() < 5) {
    frame_queue_.push_back(frame);
  }
  mutex_.unlock();
  cv_.notify_one();  // 通知录制线程有新帧可用
}

/**
 * @brief 停止录制
 * @details 停止录制线程，关闭视频写入器
 */
void Recorder::stop() {
  recoring_ = false;  ///< 设置录制标志为false
  cv_.notify_all();   ///< 唤醒录制线程以便退出
  recorder_thread_.join();  ///< 等待录制线程结束
}

/**
 * @brief 录制线程函数
 * @details 独立线程中从队列中取出帧并写入视频文件
 */
void Recorder::recorderThread() {
  while (recoring_) {  ///< 录制循环
    std::unique_lock<std::mutex> lock(mutex_);
    // 等待条件：队列有帧或停止录制
    // !recoring_ 用于在stop()调用时跳出循环
    cv_.wait(lock, [this] { return !frame_queue_.empty() || !recoring_; });
    if (!recoring_) {
      break;  ///< 如果停止录制，退出循环
    }
    // 从队列中取出帧数据
    auto buffer = std::move(frame_queue_.front());
    frame_queue_.pop_front();
    lock.unlock();  ///< 释放锁
    
    if (!buffer.empty()) {
      // 将字节数据转换为OpenCV Mat格式
      cv::Mat frame(size_, CV_8UC3, buffer.data());
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);  ///< RGB转BGR格式
      writer_.write(frame);  ///< 写入视频文件
    }
  }
  writer_.release();  ///< 释放视频写入器
}

}  // namespace fyt::camera_driver
