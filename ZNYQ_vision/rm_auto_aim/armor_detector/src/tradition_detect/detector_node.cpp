// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/tradition_detect/armor.hpp"
#include "armor_detector/tradition_detect/detector_node.hpp"

namespace armor_detector
{
namespace tradition_detect
{
/**
 * @brief 装甲板检测节点构造函数
 * @param options ROS2节点选项
 * @details 初始化所有订阅者、发布者和检测器
 */
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // 初始化检测器
  detector_ = initDetector();

  // 创建装甲板结果发布者
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // 初始化可视化标记发布者
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 初始化文本标记（用于显示分类结果）
  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 创建标记发布者
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // 初始化调试发布者
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // 初始化任务订阅者
  is_aim_task_ = true;
  task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/task_mode", 10, std::bind(&ArmorDetectorNode::taskCallback, this, std::placeholders::_1));

  // 初始化调试参数监视器
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

  // 初始化相机信息订阅者
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      // 存储相机中心点
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      // 初始化PnP求解器
      pnp_solver_ = std::make_unique<rm_auto_aim::PnPSolver>(camera_info->k, camera_info->d);
      // 相机信息获取后重置订阅者
      cam_info_sub_.reset();
    });

  // 初始化图像订阅者
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
}

/**
 * @brief 任务模式回调函数
 * @param task_msg 任务模式消息
 * @details 根据任务模式更新is_aim_task_标志
 */
void ArmorDetectorNode::taskCallback(const std_msgs::msg::String::SharedPtr task_msg)
{
  std::string task_mode = task_msg->data;
  // 根据任务模式设置标志位
  if (task_mode == "aim") {
    is_aim_task_ = true;
  } else {
    is_aim_task_ = false;
  }
}

/**
 * @brief 图像回调函数
 * @param img_msg 输入图像消息
 * @details 处理图像并发布检测结果
 */
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // 检测装甲板
  auto armors = detectArmors(img_msg);

  // 如果PnP求解器可用且处于瞄准任务模式，则进行3D位姿估计
  if (pnp_solver_ != nullptr && is_aim_task_) {
    // 初始化消息头部
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    auto_aim_interfaces::msg::Armor armor_msg;
    // 处理每个检测到的装甲板
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      // 使用PnP算法求解3D位姿
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success) {
        // 填充基本信息
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // 填充位姿信息
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        
        // 旋转向量转换为旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // 旋转矩阵转换为四元数
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // 填充到图像中心的距离
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // 填充关键点（装甲板的四个角点）
        armor_msg.kpts.clear();
        for (const auto & pt :
             {armor.left_light.top, armor.left_light.bottom, armor.right_light.bottom,
              armor.right_light.top}) {
          geometry_msgs::msg::Point point;
          point.x = pt.x;
          point.y = pt.y;
          armor_msg.kpts.emplace_back(point);
        }

        // 填充可视化标记
        armor_marker_.id++;
        // 根据装甲板类型设置不同尺寸
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        
        // 设置文本标记
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        
        // 添加到消息中
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // 发布检测到的装甲板
    armors_pub_->publish(armors_msg_);

    // 发布可视化标记
    publishMarkers();
  }
}

/**
 * @brief 初始化检测器
 * @return 初始化好的检测器对象
 * @details 从参数服务器加载检测参数并创建检测器实例
 */
std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  // 设置参数描述符
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  // 获取二值化阈值
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  // 获取检测颜色参数
  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  // 获取灯条参数
  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 35.0),
    .min_fill_ratio = declare_parameter("light.min_fill_ratio", 0.8),
  };

  // 获取装甲板参数
  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  // 创建检测器实例
  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // 初始化数字分类器
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/models/tradition_detect/mlp.onnx";
  auto label_path = pkg_path + "/models/tradition_detect/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  // 创建数字分类器实例
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

/**
 * @brief 检测装甲板
 * @param img_msg 输入图像消息
 * @return 检测到的装甲板列表
 * @details 执行完整的装甲板检测流程，包括图像预处理、参数更新、检测和调试信息发布
 */
std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // 将ROS图像消息转换为OpenCV格式
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // 更新检测器参数
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  // 执行装甲板检测
  auto armors = detector_->detect(img);

  // 计算检测延迟
  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // 发布调试信息
  if (debug_) {
    // 发布二值化图像
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // 按x坐标排序灯条和装甲板数据
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });

    // 发布灯条和装甲板调试数据
    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    // 发布数字图像
    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    // 绘制检测结果
    detector_->drawResults(img);
    // 绘制相机中心点
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // 绘制延迟信息
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    // 发布结果图像
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  return armors;
}

/**
 * @brief 创建调试发布者
 * @details 创建用于发布调试信息的发布者
 */
void ArmorDetectorNode::createDebugPublishers()
{
  // 创建灯条调试数据发布者
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  // 创建装甲板调试数据发布者
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  // 创建图像发布者
  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

/**
 * @brief 销毁调试发布者
 * @details 清理调试发布者资源
 */
void ArmorDetectorNode::destroyDebugPublishers()
{
  // 重置数据发布者
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  // 关闭图像发布者
  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

/**
 * @brief 发布可视化标记
 * @details 根据检测结果发布或删除标记
 */
void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  // 根据装甲板列表是否为空决定添加或删除标记
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace tradition_detect
}  // namespace armor_detector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::tradition_detect::ArmorDetectorNode)
