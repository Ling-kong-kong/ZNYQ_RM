// Copyright 2023 Yunlong Feng
// Copyright 2025 Lihan Chen
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

#ifndef PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
#define PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_

/**
 * @file projectile_motion_node.hpp
 * @brief 弹道解算节点头文件
 * @details 该头文件定义了弹道解算节点类，用于计算弹道轨迹并生成云台控制指令
 * 
 * 主要功能：
 * 1. 接收目标位置信息
 * 2. 考虑重力、空气阻力等物理因素计算弹道
 * 3. 生成云台控制指令以瞄准目标
 * 4. 发布射击指令
 * 5. 可视化瞄准信息
 */

#include <cstdint>
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"
#include "rmoss_projectile_motion/projectile_solver_interface.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace projectile_motion
{

/**
 * @brief 弹道解算节点类
 * @details 该节点负责接收目标位置信息，计算弹道轨迹，并生成云台控制指令
 * 
 * 工作流程：
 * 1. 接收跟踪器发布的目标信息
 * 2. 获取当前云台角度
 * 3. 考虑物理因素（重力、空气阻力等）计算弹道
 * 4. 生成云台控制指令以瞄准预测命中点
 * 5. 发布射击指令
 * 6. 可视化瞄准信息
 * 
 * 物理模型：
 * - 考虑重力加速度
 * - 考虑空气阻力
 * - 考虑发射延迟
 * - 考虑坐标系变换
 */
class ProjectileMotionNode : public rclcpp::Node
{
public:
  using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>;

public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   * @details 初始化弹道解算节点，包括订阅者、发布者和参数配置
   */
  explicit ProjectileMotionNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 目标信息回调函数
   * @param msg 目标信息消息
   * @details 处理接收到的目标信息，计算弹道并生成控制指令
   */
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  
  /**
   * @brief 发布云台控制指令
   * @param hit_pitch 命中所需的俯仰角
   * @param hit_yaw 命中所需的偏航角
   * @param shoot 射击指令
   * @param stamp 时间戳
   * @details 将计算出的云台角度发布为控制指令
   */
  void publishGimbalCommand(double hit_pitch, double hit_yaw, uint8_t shoot, rclcpp::Time stamp);
  
  /**
   * @brief 计算目标位置
   * @param msg 目标信息消息
   * @param center_position 目标中心位置
   * @param center_velocity 目标中心速度
   * @param cur_yaw 当前偏航角
   * @param hit_yaw 输出：命中所需的偏航角
   * @param hit_pitch 输出：命中所需的俯仰角
   * @details 根据目标位置和速度计算预测命中点
   */
  void calculateTargetPosition(
    const auto_aim_interfaces::msg::Target::SharedPtr & msg,
    const Eigen::Vector3d & center_position, const Eigen::Vector3d & center_velocity,
    double cur_yaw, double & hit_yaw, double & hit_pitch);
  
  /**
   * @brief 获取当前云台角度
   * @param frame_id 坐标系ID
   * @param stamp 时间戳
   * @param cur_roll 输出：当前横滚角
   * @param cur_pitch 输出：当前俯仰角
   * @param cur_yaw 输出：当前偏航角
   * @return 是否成功获取云台角度
   * @details 通过TF2获取当前云台的姿态角度
   */
  bool getCurrentGimbalAngle(
    const std::string & frame_id, const rclcpp::Time & stamp, double & cur_roll, double & cur_pitch,
    double & cur_yaw);
  
  /**
   * @brief 发布瞄准标记
   * @param hit_yaw 命中所需的偏航角
   * @param hit_pitch 命中所需的俯仰角
   * @details 发布可视化标记以显示瞄准方向
   */
  void publishHitYawMarker(double hit_yaw, double hit_pitch);

private:
  // 坐标系偏移参数
  double offset_x_;        // X轴偏移量（米）
  double offset_y_;        // Y轴偏移量（米）
  double offset_z_;        // Z轴偏移量（米）
  double offset_pitch_;    // 俯仰角偏移量（弧度）
  double offset_yaw_;      // 偏航角偏移量（弧度）
  double offset_time_;     // 时间偏移量（秒）
  
  // 弹道参数
  double shoot_speed_;     // 发射速度（米/秒）
  double friction_{0.001}; // 空气阻力系数

  // 当前云台角度
  double cur_pitch_;       // 当前俯仰角（弧度）
  double cur_yaw_;         // 当前偏航角（弧度）
  double cur_roll_;        // 当前横滚角（弧度）

  // 话题名称
  std::string target_topic_;     // 目标话题名称
  std::string gimbal_cmd_topic_; // 云台指令话题名称
  std::string shoot_cmd_topic_;  // 射击指令话题名称
  std::string solver_type_;      // 解算器类型
  std::string shooter_frame_;    // 射手坐标系名称

  // 解算器和通信接口
  std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver_;              // 弹道解算器
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_subscriber_;      // 目标订阅者
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_publisher_;  // 云台指令发布者
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_publisher_;         // 射击指令发布者

  // TF2相关
  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_sub_; // 目标消息过滤器订阅者
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                              // TF2缓冲区
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                // TF2监听器
  std::shared_ptr<tf2_filter> tf_filter_;                                  // TF2消息过滤器

  // 可视化
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aiming_marker_publisher_; // 瞄准标记发布者
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
