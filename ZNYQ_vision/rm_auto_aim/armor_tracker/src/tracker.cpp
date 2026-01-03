// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
/**
 * @brief 装甲板跟踪器构造函数
 * @param max_match_distance 最大匹配距离阈值（单位：米）
 * @param max_match_yaw_diff_ 最大匹配偏航角差阈值（单位：弧度）
 * @details 初始化跟踪器状态和参数
 *          
 *          跟踪器设计原理：
 *          1. 使用扩展卡尔曼滤波器跟踪机器人中心的运动
 *          2. 根据机器人中心和装甲板相对位置计算装甲板位置
 *          3. 通过状态机管理跟踪状态，处理目标丢失和切换
 *          
 *          状态向量定义：
 *          [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
 *          - xc, yc, za: 机器人中心坐标
 *          - vxc, vyc, vza: 机器人中心速度
 *          - yaw: 机器人偏航角
 *          - vyaw: 机器人偏航角速度
 *          - r: 机器人中心到装甲板的距离
 */
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff_)
: tracker_state(LOST),                    // 初始状态为丢失
  tracked_id(std::string("")),           // 初始无跟踪目标
  measurement(Eigen::VectorXd::Zero(4)),  // 测量向量初始化为0
  target_state(Eigen::VectorXd::Zero(9)), // 目标状态向量初始化为0
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff_)
{
  // 初始化计数器
  detect_count_ = 0;  // 检测计数器
  lost_count_ = 0;    // 丢失计数器
  change_count_ = 0;  // 切换计数器
}

/**
 * @brief 初始化跟踪器
 * @param armors_msg 装甲板消息
 * @details 选择距离图像中心最近的装甲板作为跟踪目标，初始化扩展卡尔曼滤波器
 *          
 *          初始化策略：
 *          1. 选择距离图像中心最近的装甲板作为初始目标
 *          2. 根据装甲板位置初始化机器人中心状态
 *          3. 设置跟踪状态为DETECTING，需要连续检测才能进入跟踪状态
 *          
 *          注意事项：
 *          - 初始状态假设机器人中心在装甲板后方0.2米处
 *          - 初始速度设为0，等待后续测量更新
 *          - 机器人半径初始值为0.2米
 */
void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // 选择距离图像中心最近的装甲板作为跟踪目标
  // 这种策略假设最近的装甲板是最可能的目标
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  // 初始化扩展卡尔曼滤波器
  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  // 设置跟踪ID和状态
  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;  // 设置为检测状态，需要连续检测确认

  // 初始化切换计数器
  change_count_ = 0;
  change_thres = 20;

  // 更新装甲板数量信息，判断机器人类型
  updateArmorsNum(tracked_armor);
}

/**
 * @brief 切换目标时初始化跟踪器
 * @param armor_msg 新的装甲板消息
 * @details 当需要切换跟踪目标时，重新初始化扩展卡尔曼滤波器
 *          
 *          切换目标场景：
 *          1. 用户通过服务请求切换目标
 *          2. 当前目标丢失，需要跟踪其他目标
 *          3. 策略改变，需要跟踪优先级更高的目标
 *          
 *          切换策略：
 *          1. 重置跟踪器状态
 *          2. 使用新目标初始化扩展卡尔曼滤波器
 *          3. 进入CHANGE_TARGET状态，等待稳定切换
 */
void Tracker::initChange(const Armor & armor_msg)
{
  // 初始化扩展卡尔曼滤波器
  initEKF(armor_msg);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  // 设置新的跟踪ID和状态
  tracked_id = armor_msg.number;
  tracker_state = DETECTING;  // 重新进入检测状态

  // 重置切换计数器
  change_count_ = 0;
  change_thres = 20;

  // 更新装甲板数量信息
  updateArmorsNum(armor_msg);
}

/**
 * @brief 更新跟踪器
 * @param armors_msg 装甲板消息
 * @details 执行完整的跟踪流程：预测、匹配、更新和状态机管理
 */
void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  // 扩展卡尔曼滤波器预测步骤
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  bool matched = false;
  // 如果没有找到匹配的装甲板，使用预测状态作为默认目标状态
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    // 查找相同ID的最近装甲板
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double diff_min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    int diff_count = 0;
    Armor diff_tracked_armor;
    
    // 遍历所有装甲板
    for (const auto & armor : armors_msg->armors) {
      // 只考虑相同ID的装甲板
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        // 计算预测位置与当前装甲板位置的差异
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) {
          // 找到最近的装甲板
          min_position_diff = position_diff;
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;
        }
      } else if (tracker_state == CHANGE_TARGET) {
        // 统计不同ID的装甲板数量
        diff_count += 1;
        // 计算预测位置与当前装甲板位置的差异
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < diff_min_position_diff) {
          // 找到最近的装甲板
          diff_min_position_diff = position_diff;
          diff_tracked_armor = armor;
        }
      }
    }
    
    // 如果有不同ID的装甲板，切换目标
    if (diff_count != 0) {
      initChange(diff_tracked_armor);
      return;
    }

    // 存储跟踪信息
    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    // 检查最近装甲板的距离和偏航角差是否在阈值内
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      // 找到匹配的装甲板
      matched = true;
      auto p = tracked_armor.pose.position;
      // 更新扩展卡尔曼滤波器
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      // 没有找到匹配的装甲板，但只有一个相同ID的装甲板且偏航角发生跳变
      // 认为目标正在旋转，装甲板发生跳变
      handleArmorJump(same_id_armor);
    } else {
      // 没有找到匹配的装甲板
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
    }
  }

  // 防止半径扩散过大
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // 跟踪状态机
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  } else if (tracker_state == CHANGE_TARGET) {
    if (change_count_ > change_thres) {
      tracker_state = TRACKING;
      change_count_ = 0;
    } else {
      change_count_++;
    }
  }
}

/**
 * @brief 初始化扩展卡尔曼滤波器
 * @param a 装甲板消息
 * @details 根据装甲板位置和朝向初始化目标状态向量
 *          
 *          初始化原理：
 *          1. 假设机器人中心在装甲板后方一定距离处
 *          2. 根据装甲板朝向计算机器人中心位置
 *          3. 初始速度设为0，等待后续测量更新
 *          
 *          几何关系：
 *          机器人中心 = 装甲板位置 + 半径 * 偏航角方向向量
 *          
 *          状态向量结构：
 *          [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
 *          - xc, yc, za: 机器人中心坐标
 *          - vxc, vyc, vza: 机器人中心速度（初始为0）
 *          - yaw: 机器人偏航角
 *          - vyaw: 机器人偏航角速度（初始为0）
 *          - r: 机器人中心到装甲板的距离（初始为0.2米）
 */
void Tracker::initEKF(const Armor & a)
{
  // 获取装甲板位置
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  // 获取装甲板朝向（连续变化的偏航角）
  double yaw = orientationToYaw(a.pose.orientation);

  // 在目标后方0.2米处设置初始机器人中心位置
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.2;  // 初始半径设为0.2米
  double xc = xa + r * cos(yaw);  // 机器人中心x坐标
  double yc = ya + r * sin(yaw);  // 机器人中心y坐标
  dz = 0, another_r = r;         // 高度差和另一个半径（用于4装甲板机器人）
  
  // 状态向量：[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  // 设置滤波器状态
  ekf.setState(target_state);
}

/**
 * @brief 更新装甲板数量信息
 * @param armor 装甲板消息
 * @details 根据装甲板类型和编号确定装甲板数量
 *          
 *          RoboMaster机器人类型与装甲板数量关系：
 *          1. 平衡步兵（大装甲板，编号3、4、5）：2个装甲板
 *          2. 前哨站：3个装甲板
 *          3. 普通机器人（包括英雄、步兵、哨兵等）：4个装甲板
 *          
 *          装甲板数量对跟踪的影响：
 *          1. 影响装甲板跳变处理逻辑
 *          2. 影响可视化标记的生成
 *          3. 影响半径和高度差的计算
 */
void Tracker::updateArmorsNum(const Armor & armor)
{
  // 平衡步兵（大装甲板，编号3、4、5）有2个装甲板
  // 平衡步兵是特殊的步兵类型，只有2个装甲板
  if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } 
  // 前哨站有3个装甲板
  // 前哨站是固定目标，有3个装甲板
  else if (tracked_id == "outpost") {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } 
  // 普通机器人有4个装甲板
  // 包括英雄、步兵、哨兵等大多数机器人类型
  else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

/**
 * @brief 处理装甲板跳变
 * @param current_armor 当前装甲板消息
 * @details 当目标旋转导致装甲板跳变时，更新状态并重置滤波器
 *          
 *          装甲板跳变场景：
 *          1. 目标机器人旋转，不同装甲板进入视野
 *          2. 装甲板位置和朝向发生突变
 *          3. 偏航角检测到跳变（超过阈值）
 *          
 *          处理策略：
 *          1. 更新偏航角和机器人类型信息
 *          2. 对于4装甲板机器人，交换半径和高度差
 *          3. 检查滤波器是否发散，必要时重置状态
 *          
 *          滤波器发散判断：
 *          预测装甲板位置与实际位置差值超过最大匹配距离
 */
void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  // 更新偏航角为当前装甲板的朝向
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  
  // 只有4装甲板的机器人有2个半径和高度
  // 当装甲板跳变时，可能是旋转到另一侧的装甲板
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    // 计算高度差
    dz = target_state(4) - current_armor.pose.position.z;
    // 更新机器人中心高度
    target_state(4) = current_armor.pose.position.z;
    // 交换半径，因为可能是另一侧的装甲板
    std::swap(target_state(8), another_r);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // 检查滤波器是否发散
  // 如果位置差大于最大匹配距离，认为滤波器发散，重置状态
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    // 重置状态向量，保持当前半径但重置速度
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // za
    target_state(5) = 0;                   // vza
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  // 设置滤波器状态
  ekf.setState(target_state);
}

/**
 * @brief 将四元数转换为连续变化的偏航角
 * @param q 四元数
 * @return 连续变化的偏航角
 * @details 将偏航角从(-pi~pi)转换为连续变化范围(-inf~inf)
 *          
 *          连续化处理原理：
 *          1. 标准四元数转欧拉角得到的偏航角在(-pi~pi)范围内
 *          2. 当偏航角跨越pi或-pi边界时，会产生跳变
 *          3. 通过计算最短角距离，实现角度的连续变化
 *          
 *          应用场景：
 *          1. 扩展卡尔曼滤波器需要连续的状态变量
 *          2. 检测装甲板跳变需要连续的偏航角变化
 *          3. 计算角速度需要连续的角度值
 */
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // 获取装甲板偏航角
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  
  // 使偏航角连续变化（从-pi~pi转换为-inf~inf）
  // 使用最短角距离计算角度变化，避免跳变
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

/**
 * @brief 从状态向量获取装甲板位置
 * @param x 状态向量
 * @return 装甲板位置
 * @details 根据目标中心和半径计算当前装甲板的预测位置
 *          
 *          几何关系：
 *          装甲板位置 = 机器人中心位置 - 半径 * 偏航角方向向量
 *          
 *          坐标变换：
 *          xa = xc - r * cos(yaw)
 *          ya = yc - r * sin(yaw)
 *          za = za (高度不变)
 *          
 *          应用场景：
 *          1. 预测装甲板位置用于匹配
 *          2. 计算测量残差
 *          3. 检测装甲板跳变
 */
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // 从状态向量提取机器人中心和半径信息
  double xc = x(0), yc = x(2), za = x(4);  // 机器人中心坐标
  double yaw = x(6), r = x(8);             // 偏航角和半径
  
  // 计算装甲板位置（在机器人前方）
  double xa = xc - r * cos(yaw);  // 装甲板x坐标
  double ya = yc - r * sin(yaw);  // 装甲板y坐标
  
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace rm_auto_aim
