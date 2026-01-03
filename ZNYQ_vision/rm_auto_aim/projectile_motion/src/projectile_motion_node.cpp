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

/**
 * @file projectile_motion_node.cpp
 * @brief 弹道解算节点实现文件
 * @details 该文件实现了弹道解算节点，用于计算弹道轨迹并生成云台控制指令
 */

#include "projectile_motion/projectile_motion_node.hpp"

#include "rmoss_projectile_motion/gaf_projectile_solver.hpp"
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace projectile_motion
{
/**
 * @brief 构造函数
 * @param options ROS2节点选项
 * @details 初始化弹道解算节点，包括参数配置、解算器创建、订阅者和发布者设置
 */
ProjectileMotionNode::ProjectileMotionNode(const rclcpp::NodeOptions & options)
: Node("projectile_motion_node", options)
{
  // 参数初始化
  // 角度偏移参数用于校准云台控制精度
  offset_pitch_ = this->declare_parameter("projectile.offset_pitch", 0.0);  // 俯仰角偏移（弧度）
  offset_yaw_ = this->declare_parameter("projectile.offset_yaw", 0.0);      // 偏航角偏移（弧度）
  offset_time_ = this->declare_parameter("projectile.offset_time", 0.0);    // 时间偏移（秒）
  
  // 弹道参数
  shoot_speed_ = this->declare_parameter("projectile.initial_speed", 18.0);  // 初始速度（米/秒）
  
  // 话题名称配置
  target_topic_ = this->declare_parameter("projectile.target_topic", "tracker/target");     // 目标话题
  gimbal_cmd_topic_ = this->declare_parameter("projectile.gimbal_cmd_topic", "gimbal_cmd");   // 云台指令话题
  shoot_cmd_topic_ = this->declare_parameter("projectile.shoot_cmd_topic", "cmd_shoot");       // 射击指令话题
  
  // 坐标系和解算器类型
  shooter_frame_ = this->declare_parameter("projectile.target_frame", "shooter_link");          // 射手坐标系
  solver_type_ = this->declare_parameter("projectile.solver_type", "gravity");                  // 解算器类型

  RCLCPP_INFO(this->get_logger(), "Projectile motion solver type: %s", solver_type_.c_str());
  
  // 根据类型创建弹道解算器
  // 支持两种解算器：
  // 1. gravity: 仅考虑重力的简单弹道模型
  // 2. gaf: 考虑重力和空气阻力的复杂弹道模型
  if (solver_type_ == "gravity") {
    // 重力弹道解算器，适用于低速、短距离情况
    solver_ = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    // 重力-空气阻力弹道解算器，适用于高速、远距离情况
    friction_ = this->declare_parameter("projectile.friction", 0.001);  // 空气阻力系数
    solver_ =
      std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(shoot_speed_, friction_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
    return;
  }

  // 再次确保 solver 使用的初速度是最新的
  // 防止参数服务器更新后没有重新初始化
  shoot_speed_ = get_parameter("projectile.initial_speed").as_double();
  if (solver_type_ == "gravity") {
    auto solver_ptr =
      std::dynamic_pointer_cast<rmoss_projectile_motion::GravityProjectileSolver>(solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    auto solver_ptr =
      std::dynamic_pointer_cast<rmoss_projectile_motion::GafProjectileSolver>(solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
  }

  // 初始化发布器
  // 云台控制指令（pitch/yaw）：控制云台指向目标
  gimbal_cmd_publisher_ =
    this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>(gimbal_cmd_topic_, 10);
  // 射击命令（是否开火）：控制发射器开火
  shoot_cmd_publisher_ =
    this->create_publisher<example_interfaces::msg::UInt8>(shoot_cmd_topic_, 10);
  // 可视化 marker（在 RViz 中显示瞄准箭头）：用于调试和可视化
  aiming_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("aiming_marker", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
  // 只有当目标消息能被转换到“射手坐标系（shooter_frame_）”时，才会回调 targetCallback
  // 这就避免了 TF 还没准备好就乱算的情况
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  target_sub_.subscribe(this, target_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  tf_filter_ = std::make_shared<tf2_filter>(
    target_sub_, *tf_buffer_, shooter_frame_, 50, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&ProjectileMotionNode::targetCallback, this);

  RCLCPP_INFO(this->get_logger(), "Projectile motion node initialized.");
}

void ProjectileMotionNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 没跟踪到 → 停火
  // 如果跟踪器没有跟踪到目标，发送停火指令
  if (!msg->tracking) {
    example_interfaces::msg::UInt8 shoot_cmd;
    shoot_cmd.data = 0;  // 0表示停火
    shoot_cmd_publisher_->publish(shoot_cmd);
    return;
  }

  // 获取当前云台姿态和相对位移
  // 通过TF2获取当前云台的姿态角度
  if (!getCurrentGimbalAngle(
        msg->header.frame_id, msg->header.stamp, cur_roll_, cur_pitch_, cur_yaw_)) {
    return;  // 获取失败则退出
  }

  // 构造目标中心位置和速度
  // 采用目标话题中的信息，并通过偏移量将目标中心位置转换为相对于枪管坐标系的位置
  Eigen::Vector3d center_position(
    msg->position.x + offset_x_, msg->position.y + offset_y_, msg->position.z + offset_z_);
  Eigen::Vector3d center_velocity(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // 选择最优装甲板进行打击
  double hit_yaw = 0.0, hit_pitch = 0.0;
  // 预测哪一块装甲板在子弹飞行过程中最近/最容易命中
  // 计算出应该打击该装甲板时的pitch/yaw角度
  calculateTargetPosition(msg, center_position, center_velocity, cur_yaw_, hit_yaw, hit_pitch);
  
  // 发布计算出的云台控制指令
  publishGimbalCommand(hit_pitch, hit_yaw, 1, msg->header.stamp);  // 1表示可以开火
}

/**
 * @brief 获取当前云台角度
 * @param frame_id 坐标系ID
 * @param stamp 时间戳
 * @param cur_roll 输出：当前横滚角
 * @param cur_pitch 输出：当前俯仰角
 * @param cur_yaw 输出：当前偏航角
 * @return 是否成功获取云台角度
 * @details 通过TF2获取当前云台的姿态角度，并计算坐标系偏移
 */
bool ProjectileMotionNode::getCurrentGimbalAngle(
  const std::string & frame_id, const rclcpp::Time & stamp, double & cur_roll, double & cur_pitch,
  double & cur_yaw)
{
  try {
    // 从 shooter_frame_ 到 frame_id 的变换
    // 获取射手坐标系到目标坐标系的变换关系
    auto transform = tf_buffer_->lookupTransform(frame_id, shooter_frame_, stamp);
    
    // 提取旋转四元数
    tf2::Quaternion rot_q(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w);
    
    // 将四元数转换为欧拉角（RPY）
    tf2::Matrix3x3 rot_m(rot_q);
    rot_m.getRPY(cur_roll, cur_pitch, cur_yaw);
    
    // 计算坐标系偏移
    // 位移取反，求目标在射手系中的位置
    offset_x_ = -transform.transform.translation.x;
    offset_y_ = -transform.transform.translation.y;
    offset_z_ = -transform.transform.translation.z;
    
    return true;
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return false;
  }
}

/**
 * @brief 计算预测的击打角度
 * @param msg 目标信息消息
 * @param center_position 目标中心位置
 * @param center_velocity 目标中心速度
 * @param cur_yaw 当前偏航角
 * @param hit_yaw 输出：命中所需的偏航角
 * @param hit_pitch 输出：命中所需的俯仰角
 * @details 计算最优装甲板的预测命中点，并计算所需的云台角度
 * 
 * 算法流程：
 * 1. 遍历所有装甲板，预测其在子弹飞行时间后的位置
 * 2. 选择距离最近的装甲板作为目标
 * 3. 计算击中该装甲板所需的云台角度
 * 
 * 物理模型：
 * - 考虑装甲板绕中心旋转的运动
 * - 考虑目标中心的线速度运动
 * - 考虑不同装甲板的高度差
 */
void ProjectileMotionNode::calculateTargetPosition(
  const auto_aim_interfaces::msg::Target::SharedPtr & msg, const Eigen::Vector3d & center_position,
  const Eigen::Vector3d & center_velocity, double cur_yaw, double & hit_yaw, double & hit_pitch)
{
  // 目标机器人有多个装甲板（armors_num）
  // 这些装甲板绕一个中心旋转（yaw是当前角度，v_yaw是角速度）
  // 装甲板不一定在同一高度，可能有上装甲板和下装甲板
  // 因此需要考虑radius_1、radius_2和dz参数

  // 相关参数初始化
  bool is_current_pair = true;
  double r = 0., target_dz = 0., fly_time = 0.;
  double target_pitch, target_yaw;
  Eigen::Vector3d target_position, target_predict_position;
  std::vector<std::pair<double, int>> distances;

  // First pass: 先模拟一遍所有可能要打的装甲板的未来位置
  for (int i = 0; i < msg->armors_num; ++i) {
    double tmp_yaw = msg->yaw + i * (2 * M_PI / msg->armors_num);

    // 1) 根据是4块还是别的，决定半径和高度
    // 4 块的时候会出现大小半径/高低交替
    // 所以他用 is_current_pair 或者用 min_index % 2 那种逻辑去区分“这一块是大半径还是小半径、是上装甲板还是下装甲板”
    if (msg->armors_num == 4) {
      r = is_current_pair ? msg->radius_1 : msg->radius_2;
      is_current_pair = !is_current_pair;
      target_dz = is_current_pair ? 0. : msg->dz;
    } else {
      r = msg->radius_1;
      target_dz = 0.;
    }

    // 2) 算出当前这块装甲板的空间位置 target_position
    target_position =
      center_position + Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);
    // 3) 估个飞行时间 fly_time
    fly_time = target_position.head(2).norm() / shoot_speed_ + offset_time_;
    // 4) 用角速度把装甲板往前“转” fly_time
    // 装甲板会绕中心转动
    tmp_yaw += msg->v_yaw * fly_time;
    // 5) 把目标中心自己的线速度也加上（目标本身在动）
    target_predict_position =
      center_position + center_velocity * fly_time +
      Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);

    // 6) 得到真正“子弹到时装甲板位置” = target_predict_position
    double distance = target_predict_position.head(2).norm();
    // 7) 存一下它的水平距离 distance
    distances.push_back(std::make_pair(distance, i));
  }

  // 然后找最近的一块
  auto min_distance_it = std::min_element(distances.begin(), distances.end());
  int min_index = min_distance_it->second;

  // Second pass: 对“最近的那块”计算角度
  double tmp_yaw = msg->yaw + min_index * (2 * M_PI / msg->armors_num);

  if (msg->armors_num == 4) {
    r = (min_index % 2 == 0) ? msg->radius_1 : msg->radius_2;
    target_dz = (min_index % 2 == 0) ? 0. : msg->dz;
  } else {
    r = msg->radius_1;
    target_dz = 0.;
  }

  target_position =
    center_position + Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);
  fly_time = target_position.head(2).norm() / shoot_speed_ + offset_time_;
  tmp_yaw += msg->v_yaw * fly_time;
  target_predict_position =
    center_position + center_velocity * fly_time +
    Eigen::Vector3d(-r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw), target_dz);

  // pitch 用弹道解算器算出来
  solver_->solve(target_predict_position.head(2).norm(), target_predict_position.z(), target_pitch);
  // 这是坐标系/云台定义问题，需要反过来
  target_pitch = -target_pitch;
  // yaw 不用弹道解，直接 atan2(y, x)，因为水平转向通常不受重力影响
  target_yaw = std::atan2(target_predict_position.y(), target_predict_position.x());

  hit_yaw = target_yaw;
  hit_pitch = target_pitch;

  // 发布可视化 marker
  publishHitYawMarker(hit_yaw, hit_pitch);
}

/**
 * @brief 发布云台和射击指令
 * @param hit_pitch 命中所需的俯仰角
 * @param hit_yaw 命中所需的偏航角
 * @param shoot 射击指令
 * @param stamp 时间戳
 * @details 发布云台控制指令和射击指令，确保只有在云台角度对准时才开火
 * 
 * 工作流程：
 * 1. 构建云台控制指令消息
 * 2. 应用角度偏移校准
 * 3. 检查云台角度是否对准
 * 4. 发布指令
 */
void ProjectileMotionNode::publishGimbalCommand(
  double hit_pitch, double hit_yaw, uint8_t shoot, rclcpp::Time stamp)
{
  auto_aim_interfaces::msg::GimbalCmd gimbal_cmd;
  example_interfaces::msg::UInt8 shoot_cmd;
  
  // 设置消息头
  gimbal_cmd.header.stamp = this->now();
  gimbal_cmd.latency = (this->now() - stamp).seconds() * 1000.0;  // 计算延迟（毫秒）
  
  // 设置云台控制类型为绝对角度
  gimbal_cmd.pitch_type = auto_aim_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_cmd.yaw_type = auto_aim_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  
  // 应用角度偏移校准
  gimbal_cmd.position.pitch = hit_pitch + offset_pitch_;  // 俯仰角 + 校准偏移
  gimbal_cmd.position.yaw = hit_yaw + offset_yaw_;      // 偏航角 + 校准偏移

  // 打击闭环的准星收敛判据
  // 只有当计算出的目标角度与当前云台角度足够接近时才开火
  // 这确保了射击精度，避免在云台还在移动时开火
  shoot_cmd.data =
    (fabs(hit_pitch - cur_pitch_) < 0.1 && fabs(hit_yaw - cur_yaw_) < 0.1) ? shoot : 0;
  
  // 设置射击指令
  gimbal_cmd.shoot_cmd = shoot_cmd.data;
  
  // 发布指令
  gimbal_cmd_publisher_->publish(gimbal_cmd);
  shoot_cmd_publisher_->publish(shoot_cmd);
}

/**
 * @brief 发布可视化标记
 * @param hit_yaw 命中所需的偏航角
 * @param hit_pitch 命中所需的俯仰角
 * @details 在固定坐标系下发布一个红色箭头标记，显示瞄准方向
 * 
 * 可视化参数：
 * - 坐标系：odom（世界坐标系）
 * - 标记类型：箭头
 * - 颜色：红色
 * - 长度：5.0米
 * 
 * 用途：
 * - 在RViz中显示瞄准方向
 * - 调试弹道解算结果
 * - 验证云台控制精度
 */
void ProjectileMotionNode::publishHitYawMarker(double hit_yaw, double hit_pitch)
{
  visualization_msgs::msg::Marker marker;
  
  // 设置标记的基本属性
  marker.header.frame_id = "odom";  // 在世界坐标系中显示
  marker.header.stamp = this->now();
  marker.ns = "hit_yaw";            // 命名空间
  marker.id = 0;                    // 标记ID
  
  // 设置标记类型为箭头
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // 设置箭头方向（从RPY角度创建四元数）
  tf2::Quaternion q;
  q.setRPY(0, hit_pitch, hit_yaw);  // 横滚为0，俯仰和偏航由计算得出
  marker.pose.orientation = tf2::toMsg(q);
  
  // 设置箭头尺寸
  marker.scale.x = 5.0;   // 箭头长度（米）
  marker.scale.y = 0.01;  // 箭头宽度（米）
  marker.scale.z = 0.01;  // 箭头高度（米）
  
  // 设置箭头颜色为红色
  marker.color.a = 1.0;  // 不透明度
  marker.color.r = 1.0;  // 红色分量
  marker.color.g = 0.0;  // 绿色分量
  marker.color.b = 0.0;  // 蓝色分量

  // 发布标记
  aiming_marker_publisher_->publish(marker);
}

}  // namespace projectile_motion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(projectile_motion::ProjectileMotionNode)
