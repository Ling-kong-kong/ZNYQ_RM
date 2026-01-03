// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

/**
 * @brief 装甲板数量枚举
 * @details 定义不同类型机器人的装甲板数量
 */
enum class ArmorsNum { 
  NORMAL_4 = 4,    // 普通机器人：4个装甲板
  BALANCE_2 = 2,    // 平衡步兵：2个装甲板（大装甲板）
  OUTPOST_3 = 3     // 前哨站：3个装甲板
};

/**
 * @brief 装甲板跟踪器类
 * @details 实现对单个装甲板目标的跟踪功能，包括：
 *          1. 目标选择和初始化
 *          2. 使用扩展卡尔曼滤波器进行状态预测和更新
 *          3. 处理目标匹配和切换
 *          4. 处理装甲板跳变（目标旋转时）
 *          5. 管理跟踪状态机
 *          
 *          跟踪器状态机说明：
 *          - LOST：丢失状态，未跟踪任何目标
 *          - DETECTING：检测状态，短暂识别到目标，需要更多帧确认
 *          - TRACKING：跟踪状态，正常跟踪目标
 *          - TEMP_LOST：临时丢失状态，短暂丢失目标，使用预测值
 *          - CHANGE_TARGET：切换目标状态，正在切换到新目标
 */
class Tracker
{
public:
  /**
   * @brief 跟踪器构造函数
   * @param max_match_distance 最大匹配距离阈值（米）
   * @param max_match_yaw_diff_ 最大匹配偏航角差阈值（弧度）
   */
  Tracker(double max_match_distance, double max_match_yaw_diff_);

  // 类型别名
  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  /**
   * @brief 初始化跟踪器
   * @param armors_msg 装甲板消息
   * @details 选择距离图像中心最近的装甲板作为跟踪目标，初始化扩展卡尔曼滤波器
   */
  void init(const Armors::SharedPtr & armors_msg);

  /**
   * @brief 更新跟踪器
   * @param armors_msg 装甲板消息
   * @details 执行完整的跟踪流程：预测、匹配、更新和状态机管理
   */
  void update(const Armors::SharedPtr & armors_msg);

  /**
   * @brief 自适应调整角速度
   * @param duration 时间间隔
   * @details 根据时间间隔调整角速度估计
   */
  void adaptAngularVelocity(const double & duration);

  // 扩展卡尔曼滤波器实例
  ExtendedKalmanFilter ekf;

  // 跟踪阈值参数
  int tracking_thres;  // 从检测状态进入跟踪状态所需的连续检测帧数
  int lost_thres;      // 从临时丢失状态进入丢失状态所需的连续丢失帧数
  int change_thres;    // 切换目标所需的帧数

  /**
   * @brief 跟踪器状态枚举
   */
  enum State {
    LOST,          // 丢失状态
    DETECTING,     // 检测状态
    TRACKING,      // 跟踪状态
    TEMP_LOST,     // 临时丢失状态
    CHANGE_TARGET, // 切换目标状态
  } tracker_state;

  // 跟踪目标信息
  std::string tracked_id;        // 当前跟踪的目标ID
  std::string last_tracked_id;   // 上次跟踪的目标ID
  Armor tracked_armor;           // 当前跟踪的装甲板
  ArmorsNum tracked_armors_num;  // 跟踪目标的装甲板数量

  // 调试信息
  double info_position_diff;  // 位置差异（用于调试）
  double info_yaw_diff;       // 偏航角差异（用于调试）

  // 测量和状态向量
  Eigen::VectorXd measurement;  // 观测向量
  Eigen::VectorXd target_state; // 目标状态向量

  // 用于存储另一对装甲板信息（处理4装甲板机器人的高度差）
  double dz, another_r;

private:
  /**
   * @brief 初始化扩展卡尔曼滤波器
   * @param a 装甲板消息
   * @details 根据装甲板位置和朝向初始化目标状态向量
   */
  void initEKF(const Armor & a);

  /**
   * @brief 切换目标时初始化跟踪器
   * @param armor_msg 新的装甲板消息
   * @details 当需要切换跟踪目标时，重新初始化扩展卡尔曼滤波器
   */
  void initChange(const Armor & armor_msg);

  /**
   * @brief 更新装甲板数量信息
   * @param a 装甲板消息
   * @details 根据装甲板类型和编号确定装甲板数量
   */
  void updateArmorsNum(const Armor & a);

  /**
   * @brief 处理装甲板跳变
   * @param a 当前装甲板消息
   * @details 当目标旋转导致装甲板跳变时，更新状态并重置滤波器
   */
  void handleArmorJump(const Armor & a);

  /**
   * @brief 将四元数转换为偏航角
   * @param q 四元数
   * @return 连续变化的偏航角
   * @details 将偏航角从(-pi~pi)转换为连续变化范围(-inf~inf)
   */
  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  /**
   * @brief 从状态向量获取装甲板位置
   * @param x 状态向量
   * @return 装甲板位置
   * @details 根据目标中心和半径计算当前装甲板的预测位置
   */
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  // 匹配参数
  double max_match_distance_;    // 最大匹配距离阈值
  double max_match_yaw_diff_;    // 最大匹配偏航角差阈值

  // 状态计数器
  int detect_count_;  // 检测计数器
  int lost_count_;    // 丢失计数器
  int change_count_;  // 切换计数器

  // 上一次的偏航角（用于连续化处理）
  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
