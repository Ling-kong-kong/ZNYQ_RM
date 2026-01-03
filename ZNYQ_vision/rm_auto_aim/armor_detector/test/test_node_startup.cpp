// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

// STD
#include <memory>

#include "armor_detector/tradition_detect/detector_node.hpp"

/**
 * @brief 装甲板检测节点启动测试
 * @details 测试节点是否能够正常启动和关闭
 *          这是一个基本的单元测试，验证节点的初始化和销毁过程
 */
TEST(ArmorDetectorNodeTest, NodeStartupTest)
{
  // 创建节点选项
  rclcpp::NodeOptions options;
  // 创建装甲板检测节点实例
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(options);
  // 重置节点，测试正常销毁过程
  node.reset();
}

/**
 * @brief 测试主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数列表
 * @return 测试结果
 * @details 初始化Google Test框架和ROS2，运行所有测试，并清理资源
 */
int main(int argc, char ** argv)
{
  // 初始化Google Test框架
  testing::InitGoogleTest(&argc, argv);
  // 初始化ROS2
  rclcpp::init(argc, argv);
  // 运行所有测试
  auto result = RUN_ALL_TESTS();
  // 关闭ROS2
  rclcpp::shutdown();
  return result;
}
