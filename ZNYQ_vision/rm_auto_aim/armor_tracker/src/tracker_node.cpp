// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker_node.hpp"

// STD
#include <cmath>
#include <memory>
#include <vector>

namespace rm_auto_aim
{
/**
 * @brief 装甲板跟踪节点构造函数
 * @param options ROS2节点选项
 * @details 初始化所有订阅者、发布者、服务和扩展卡尔曼滤波器
 *          
 *          节点功能概述：
 *          1. 订阅装甲板检测结果，进行坐标变换
 *          2. 使用扩展卡尔曼滤波器跟踪装甲板目标
 *          3. 发布跟踪结果和可视化标记
 *          4. 提供重置和切换目标的服务接口
 *          
 *          扩展卡尔曼滤波器配置：
 *          - 状态向量：[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
 *          - 观测向量：[xa, ya, za, yaw]
 *          - 运动模型：匀速运动模型
 *          - 几何模型：装甲板在机器人前方
 */
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // XOY平面中允许的最大装甲板距离
  // 用于过滤异常检测结果，避免跟踪过远的目标
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // 初始化跟踪器
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff_ = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff_);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // 初始化扩展卡尔曼滤波器(EKF)
  // xa = x_armor, xc = x_robot_center
  // 状态向量: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // 观测向量: xa, ya, za, yaw
  
  // f - 过程函数：根据当前状态预测下一时刻状态
  // 使用匀速运动模型，假设目标在短时间内保持匀速运动
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    // 位置更新：新位置 = 旧位置 + 速度 * 时间间隔
    x_new(0) += x(1) * dt_;  // xc += v_xc * dt  (机器人中心x位置)
    x_new(2) += x(3) * dt_;  // yc += v_yc * dt  (机器人中心y位置)
    x_new(4) += x(5) * dt_;  // za += v_za * dt  (机器人中心z位置)
    x_new(6) += x(7) * dt_;  // yaw += v_yaw * dt (偏航角)
    // 速度和半径保持不变（匀速运动模型）
    return x_new;
  };
  // J_f - 过程函数的雅可比矩阵
  // 表示状态向量对自身的偏导数，用于线性化非线性状态转移函数
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    // 状态向量对自身的偏导数矩阵
    // 行表示新状态分量，列表示旧状态分量
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,   // ∂xc/∂xc, ∂xc/∂v_xc
          0,   1,   0,   0,   0,   0,   0,   0,   0,   // ∂v_xc/∂v_xc
          0,   0,   1,   dt_, 0,   0,   0,   0,   0,   // ∂yc/∂yc, ∂yc/∂v_yc
          0,   0,   0,   1,   0,   0,   0,   0,   0,   // ∂v_yc/∂v_yc
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,   // ∂za/∂za, ∂za/∂v_za
          0,   0,   0,   0,   0,   1,   0,   0,   0,   // ∂v_za/∂v_za
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,   // ∂yaw/∂yaw, ∂yaw/∂v_yaw
          0,   0,   0,   0,   0,   0,   0,   1,   0,   // ∂v_yaw/∂v_yaw
          0,   0,   0,   0,   0,   0,   0,   0,   1;   // ∂r/∂r (半径不变)
    // clang-format on
    return f;
  };
  // h - 观测函数：根据状态向量计算观测值
  // 描述了从机器人中心状态到装甲板观测值的几何关系
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    // 根据机器人中心位置、偏航角和半径计算装甲板位置
    // 几何关系：装甲板在机器人前方r距离处
    z(0) = xc - r * cos(yaw);  // xa: 装甲板x坐标
    z(1) = yc - r * sin(yaw);  // ya: 装甲板y坐标
    z(2) = x(4);               // za: 装甲板z坐标（与机器人中心相同）
    z(3) = x(6);               // yaw: 装甲板偏航角（与机器人相同）
    return z;
  };
  // J_h - 观测函数的雅可比矩阵
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    // 观测值对状态向量的偏导数矩阵
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),  // ∂xa/∂xc, ∂xa/∂yaw, ∂xa/∂r
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),  // ∂ya/∂yc, ∂ya/∂yaw, ∂ya/∂r
          0,   0,   0,   0,   1,   0,   0,          0,   0,          // ∂za/∂za
          0,   0,   0,   0,   0,   0,   1,          0,   0;          // ∂yaw/∂yaw
    // clang-format on
    return h;
  };
  // update_Q - 过程噪声协方差矩阵更新函数
  s2qxyz_max_ = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
  s2qxyz_min_ = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
  s2qyaw_max_ = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
  s2qyaw_min_ = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 80.0);
  auto u_q = [this](const Eigen::VectorXd & x_p) {
    // 提取速度信息
    double vx = x_p(1), vy = x_p(3), v_yaw = x_p(7);
    // 计算线速度和角速度大小
    double dx = pow(pow(vx, 2) + pow(vy, 2), 0.5);
    double dy = abs(v_yaw);
    Eigen::MatrixXd q(9, 9);
    double x, y;
    // 根据速度自适应调整噪声大小：速度越大，噪声越小
    // 基于RoboMaster机器人的物理特性：平移和旋转运动存在负相关关系
    x = exp(-dy) * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
    y = exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;
    double t = dt_, r = s2qr_;
    // 计算位置和速度的噪声方差（基于匀速运动模型）
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };
  // update_R - 观测噪声协方差矩阵更新函数
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    // 观测噪声与测量值成正比
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };
  
  // P - 误差估计协方差矩阵
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  // 创建扩展卡尔曼滤波器实例
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // 创建重置跟踪器服务
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      // 重置跟踪器状态为丢失
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // 创建切换目标服务
  change_target_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/change", [this](
                         const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr response) {
      // 设置跟踪器状态为切换目标
      tracker_->tracker_state = Tracker::CHANGE_TARGET;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Target change!");
      return;
    });

  // 创建带有tf2消息过滤器的订阅者
  // tf2相关配置
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // 在调用waitForTransform之前创建定时器接口，
  // 避免tf2_ros::CreateTimerInterfaceException异常
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  // 订阅者和过滤器
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // 注册回调函数，当变换可用时调用
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // 测量信息发布器（用于调试）
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // 目标信息发布器
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  // 可视化标记发布器
  // 参见 http://wiki.ros.org/rviz/DisplayTypes/Marker
  
  // 位置标记（球体）
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  
  // 线速度标记（箭头）
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  
  // 角速度标记（箭头）
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  
  // 装甲板标记（立方体）
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // 将装甲板位置从图像坐标系转换到世界坐标系
  // 使用tf2进行坐标系转换，确保位置信息在正确的参考系中
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // 过滤异常装甲板数据
  // 1. 过滤高度异常的装甲板（|z| > 1.2米）
  // 2. 过滤距离过远的装甲板（超过最大允许距离）
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;
    } else if (tracker_->tracker_state == Tracker::CHANGE_TARGET) {
      target_msg.tracking = false;
    }
  }

  last_time_ = time;

  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  // 设置标记的时间戳和坐标系
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    // 提取目标状态信息
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    // 设置位置标记（机器人中心）
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;  // 高度取两个装甲板的中间值

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    // 设置装甲板标记
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;  // 标记当前是否为第一对装甲板
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    
    // 根据装甲板数量生成所有装甲板标记
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);  // 计算每个装甲板的偏航角
      
      // 只有4装甲板的机器人有2个半径和高度
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;  // 交替使用两个半径
        p_a.z = za + (is_current_pair ? 0 : dz);  // 交替使用两个高度
        is_current_pair = !is_current_pair;
      } else {
        r = r1;  // 其他机器人使用单一半径
        p_a.z = za;
      }
      
      // 计算装甲板位置
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      // 设置装甲板标记属性
      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      // 前哨站有特殊倾斜角度
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
