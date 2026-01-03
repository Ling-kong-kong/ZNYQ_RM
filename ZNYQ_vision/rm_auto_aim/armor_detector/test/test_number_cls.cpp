// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>

// STL
#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <vector>

#include "armor_detector/tradition_detect/number_classifier.hpp"

// 使用高分辨率时钟进行性能测试
using hrc = std::chrono::high_resolution_clock;

/**
 * @brief 数字分类器性能基准测试
 * @details 测试数字分类器的性能指标，包括：
 *          1. 最小处理时间
 *          2. 最大处理时间
 *          3. 平均处理时间
 *          通过多次测试和预热排除性能抖动影响
 */
TEST(test_nc, benchmark)
{
  // 获取包的共享目录路径
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  // 构建模型文件路径
  auto model_path = pkg_path + "/model/mlp.onnx";
  // 构建标签文件路径
  auto label_path = pkg_path + "/model/label.txt";
  // 创建数字分类器实例，设置置信度阈值为0.5
  rm_auto_aim::NumberClassifier nc(model_path, label_path, 0.5);

  // 创建虚拟装甲板数据用于测试
  auto dummy_armors = std::vector<rm_auto_aim::Armor>(1);
  // 创建测试用的数字图像（20x28单通道）
  auto test_mat = cv::Mat(20, 28, CV_8UC1);
  dummy_armors[0].number_img = test_mat;

  // 设置测试参数
  int loop_num = 200;  // 测试循环次数
  int warm_up = 30;    // 预热次数

  // 初始化性能统计变量
  double time_min = DBL_MAX;  // 最小时间
  double time_max = -DBL_MAX; // 最大时间
  double time_avg = 0;        // 平均时间

  // 执行性能测试（包含预热阶段）
  for (int i = 0; i < warm_up + loop_num; i++) {
    // 记录开始时间
    auto start = hrc::now();
    // 执行数字分类
    nc.classify(dummy_armors);
    // 记录结束时间
    auto end = hrc::now();
    // 计算处理时间（毫秒）
    double time = std::chrono::duration<double, std::milli>(end - start).count();
    
    // 预热阶段结束后开始统计
    if (i >= warm_up) {
      time_min = std::min(time_min, time);
      time_max = std::max(time_max, time);
      time_avg += time;
    }
  }
  // 计算平均处理时间
  time_avg /= loop_num;

  // 输出性能测试结果
  std::cout << "time_min: " << time_min << "ms" << std::endl;
  std::cout << "time_max: " << time_max << "ms" << std::endl;
  std::cout << "time_avg: " << time_avg << "ms" << std::endl;
}
