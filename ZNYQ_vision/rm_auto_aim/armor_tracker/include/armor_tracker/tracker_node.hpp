// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;

/**
 * @brief 装甲板跟踪节点类
 * @details 该节点是RoboMaster自瞄系统的核心组件之一，负责：
 *          1. 订阅装甲板检测结果并进行坐标变换
 *          2. 使用扩展卡尔曼滤波器跟踪装甲板目标
 *          3. 发布跟踪结果和可视化标记
 *          4. 提供重置和切换目标的服务接口
 *          
 *          工作流程：
 *          1. 接收来自检测节点的装甲板消息
 *          2. 将装甲板位置从相机坐标系转换到目标坐标系
 *          3. 过滤异常装甲板数据
 *          4. 使用跟踪器进行目标跟踪
 *          5. 发布跟踪结果和可视化信息
 */
class ArmorTrackerNode : public rclcpp::Node
{
public:
  /**
   * @brief 装甲板跟踪节点构造函数
   * @param options ROS2节点选项
   * @details 初始化所有订阅者、发布者、服务和扩展卡尔曼滤波器
   */
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 装甲板消息回调函数
   * @param armors_ptr 装甲板消息指针
   * @details 处理装甲板检测结果，执行坐标变换和跟踪
   */
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  /**
   * @brief 发布可视化标记
   * @param target_msg 目标消息
   * @details 发布目标位置、速度和装甲板的可视化标记
   */
  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // 参数配置
  double max_armor_distance_;  // XOY平面中允许的最大装甲板距离

  // 时间相关
  rclcpp::Time last_time_;  // 上次接收消息的时间
  double dt_;               // 时间间隔

  // 扩展卡尔曼滤波器参数
  double s2qxyz_max_, s2qxyz_min_;  // 过程噪声位置方差的最大值和最小值
  double s2qyaw_max_, s2qyaw_min_;  // 过程噪声偏航角方差的最大值和最小值
  double s2qr_;                     // 过程噪声半径方差
  double r_xyz_factor, r_yaw;       // 观测噪声因子
  double lost_time_thres_;          // 丢失时间阈值
  
  // 跟踪器实例
  std::unique_ptr<Tracker> tracker_;

  // 服务接口
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;   // 重置跟踪器服务
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;  // 切换目标服务

  // 订阅器和tf2消息过滤器
  std::string target_frame_;                                                 // 目标坐标系
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;                            // tf2缓冲区
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;               // tf2监听器
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_; // 装甲板消息订阅者
  std::shared_ptr<tf2_filter> tf2_filter_;                                 // tf2消息过滤器

  // 发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;  // 跟踪信息发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;    // 目标信息发布者

  // 可视化标记
  visualization_msgs::msg::Marker position_marker_;  // 位置标记（球体）
  visualization_msgs::msg::Marker linear_v_marker_;  // 线速度标记（箭头）
  visualization_msgs::msg::Marker angular_v_marker_; // 角速度标记（箭头）
  visualization_msgs::msg::Marker armor_marker_;      // 装甲板标记（立方体）
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // 标记数组发布者
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
