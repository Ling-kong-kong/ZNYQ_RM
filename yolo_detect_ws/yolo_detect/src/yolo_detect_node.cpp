/**
 * @file yolo_detect_node.cpp
 * @brief YOLO检测节点实现
 * @details 订阅海康相机图像话题，使用YOLOv5+OpenVINO进行装甲板检测
 */

#include "yolo_detect/yolo_detect_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
#include "yolo_detect/ros2_armor.hpp"
#include "yolo_detect/pnp_solver.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace yolo_detect
{

  // 构造函数实现
  YoloDetect::YoloDetect() : Node("yolo_detect_node"), frame_count_(0), last_frame_time_(std::chrono::high_resolution_clock::now())
  {
    // 初始化YOLO相关参数和检测器
    if (!initYolo())
    {
      RCLCPP_ERROR(this->get_logger(), "YOLO初始化失败，节点退出");
      return;
    }

    // 创建装甲板发布器
    armors_pub_ = this->create_publisher<yolo_detect::msg::Armors>(
        "/detector/armors", rclcpp::SensorDataQoS());

    // 初始化可视化标记
    armor_marker_.ns = "armors";
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x = 0.05;
    armor_marker_.scale.z = 0.125;
    armor_marker_.color.a = 1.0;
    armor_marker_.color.g = 0.5;
    armor_marker_.color.b = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns = "classification";
    text_marker_.action = visualization_msgs::msg::Marker::ADD;
    text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z = 0.1;
    text_marker_.color.a = 1.0;
    text_marker_.color.r = 1.0;
    text_marker_.color.g = 1.0;
    text_marker_.color.b = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // 创建标记发布器
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

    // 创建任务订阅器 没啥用
    is_aim_task_ = true;
    task_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/task_mode", 10, std::bind(&YoloDetect::task_callback, this, std::placeholders::_1));

    // 创建图像订阅者
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&YoloDetect::image_callback, this, std::placeholders::_1));

    // 创建相机信息订阅器
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
        {
          cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
          cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);

          // 创建PnP求解器
          std::array<double, 9> camera_matrix;
          std::copy(camera_info->k.begin(), camera_info->k.end(), camera_matrix.begin());
          pnp_solver_ = std::make_unique<rm_auto_aim::PnPSolver>(camera_matrix, camera_info->d);

          cam_info_sub_.reset();
        });

    // 创建检测结果图像发布者
    result_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/result_img",
                                                                      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    RCLCPP_INFO(this->get_logger(), "YOLO检测节点启动成功");
  }

  /**
   * @brief 验证ROI参数是否在图像边界内
   * @param x ROI X坐标
   * @param y ROI Y坐标
   * @param width ROI宽度
   * @param height ROI高度
   * @return 是否在边界内
   */
  bool YoloDetect::validate_roi_bounds(int x, int y, int width, int height)
  {
    // 检查ROI是否超出图像边界
    bool x_valid = (x >= 0 && x < image_width_);
    bool y_valid = (y >= 0 && y < image_height_);
    bool width_valid = (width > 0 && (x + width) <= image_width_);
    bool height_valid = (height > 0 && (y + height) <= image_height_);

    return x_valid && y_valid && width_valid && height_valid;
  }

  /**
   * @brief 初始化YOLO相关参数和检测器
   * @return 初始化是否成功
   */
  bool YoloDetect::initYolo()
  {
    // 声明基本参数
    this->declare_parameter("config_path", "config/sentry.yaml");
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("image_topic", "/image_raw");

    // 获取基本参数
    config_path_ = this->get_parameter("config_path").as_string();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    image_topic_ = this->get_parameter("image_topic").as_string();

    // 处理相对路径：获取包的共享目录路径
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("yolo_detect");
    if (config_path_[0] != '/')
    {
      config_path_ = package_share_directory + "/" + config_path_;
    }

    // 从配置文件读取ROI参数的默认值
    bool config_use_roi = false;
    int config_roi_x = 420;
    int config_roi_y = 50;
    int config_roi_width = 600;
    int config_roi_height = 600;

    try
    {
      auto yaml = YAML::LoadFile(config_path_);
      config_use_roi = yaml["use_roi"].as<bool>();
      if (config_use_roi)
      {
        config_roi_x = yaml["roi"]["x"].as<int>();
        config_roi_y = yaml["roi"]["y"].as<int>();
        config_roi_width = yaml["roi"]["width"].as<int>();
        config_roi_height = yaml["roi"]["height"].as<int>();
        RCLCPP_INFO(this->get_logger(), "从配置文件加载ROI区域: (%d, %d, %d, %d)",
                    config_roi_x, config_roi_y, config_roi_width, config_roi_height);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "配置文件中ROI区域未启用");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "从配置文件加载ROI配置失败: %s，使用默认值", e.what());
    }

    // 声明ROI相关参数，使用配置文件中的值作为默认值
    this->declare_parameter("use_roi", config_use_roi);
    this->declare_parameter("roi_x", config_roi_x);
    this->declare_parameter("roi_y", config_roi_y);
    this->declare_parameter("roi_width", config_roi_width);
    this->declare_parameter("roi_height", config_roi_height);

    // 获取ROI参数（此时已经包含了配置文件的值或用户通过命令行设置的值）
    use_roi_ = this->get_parameter("use_roi").as_bool();
    int roi_x = this->get_parameter("roi_x").as_int();
    int roi_y = this->get_parameter("roi_y").as_int();
    int roi_width = this->get_parameter("roi_width").as_int();
    int roi_height = this->get_parameter("roi_height").as_int();

    // 从配置文件读取图像尺寸
    image_width_ = 1440;  // 默认值
    image_height_ = 1080; // 默认值

    try
    {
      auto yaml = YAML::LoadFile(config_path_);
      if (yaml["image_width"])
      {
        image_width_ = yaml["image_width"].as<int>();
      }
      if (yaml["image_height"])
      {
        image_height_ = yaml["image_height"].as<int>();
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "从配置文件读取图像尺寸失败，使用默认值: %s", e.what());
    }

    // 验证ROI参数是否在图像边界内
    if (!validate_roi_bounds(roi_x, roi_y, roi_width, roi_height))
    {
      RCLCPP_ERROR(this->get_logger(), "ROI参数超出图像边界: (%d, %d, %d, %d), 图像尺寸: (%d, %d)",
                   roi_x, roi_y, roi_width, roi_height, image_width_, image_height_);
      return false;
    }

    roi_ = cv::Rect(roi_x, roi_y, roi_width, roi_height);

    // 设置参数回调，允许动态更新
    RCLCPP_ERROR(this->get_logger(), "正在设置参数回调...");

    // 保存回调句柄
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
        {
          return roi_parameter_callback(parameters);
        });

    RCLCPP_ERROR(this->get_logger(), "参数回调设置完成！");

    RCLCPP_INFO(this->get_logger(), "ROI区域设置: (%d, %d, %d, %d), 状态: %s",
                roi_.x, roi_.y, roi_.width, roi_.height, use_roi_ ? "启用" : "禁用");

    if (use_roi_)
    {
      RCLCPP_INFO(this->get_logger(), "ROI区域启用: (%d, %d, %d, %d)", roi_.x, roi_.y, roi_.width, roi_.height);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "ROI区域未启用");
    }

    try
    {
      // 初始化YOLOv5检测器，传递ROI配置
      detector_ = std::make_unique<yolo_detect::YOLOV5>(config_path_, debug_mode_, use_roi_, roi_);
      RCLCPP_INFO(this->get_logger(), "YOLOv5检测器初始化成功");
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "YOLOv5检测器初始化失败: %s", e.what());
      return false;
    }
  }

  /**
   * @brief 参数回调函数
   * @param parameters 参数列表
   * @return 设置结果
   */
  rcl_interfaces::msg::SetParametersResult YoloDetect::parameter_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    static int callback_count = 0;
    callback_count++;
    RCLCPP_ERROR(this->get_logger(), "!!! 参数回调被调用，次数: %d，参数数量: %zu !!!", callback_count, parameters.size());
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      if (param.get_name() == "use_roi")
      {
        use_roi_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "ROI使用状态更新为: %s", use_roi_ ? "启用" : "禁用");
      }
      else if (param.get_name() == "roi_x")
      {
        roi_.x = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROI X坐标更新为: %d", roi_.x);
      }
      else if (param.get_name() == "roi_y")
      {
        roi_.y = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROI Y坐标更新为: %d", roi_.y);
      }
      else if (param.get_name() == "roi_width")
      {
        roi_.width = param.as_int();
        RCLCPP_WARN(this->get_logger(), "!!! ROI宽度更新为: %d !!!", roi_.width);
        RCLCPP_WARN(this->get_logger(), "!!! 当前ROI区域: (%d, %d, %d, %d) !!!", roi_.x, roi_.y, roi_.width, roi_.height);
      }
      else if (param.get_name() == "roi_height")
      {
        roi_.height = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROI高度更新为: %d", roi_.height);
      }
    }

    // 如果检测器已初始化，更新ROI设置而不是重新创建检测器
    if (detector_ && result.successful)
    {
      try
      {
        RCLCPP_INFO(this->get_logger(), "更新检测器ROI区域: (%d, %d, %d, %d)", roi_.x, roi_.y, roi_.width, roi_.height);
        detector_->update_roi(roi_, use_roi_);
        RCLCPP_INFO(this->get_logger(), "检测器ROI设置已更新");
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "更新检测器ROI失败: %s", e.what());
        result.successful = false;
      }
    }

    return result;
  }

  /**
   * @brief ROI参数回调处理函数
   * @param parameters 参数列表
   * @return 设置结果
   */
  rcl_interfaces::msg::SetParametersResult YoloDetect::roi_parameter_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    static int callback_count = 0;
    callback_count++;
    RCLCPP_ERROR(this->get_logger(), "!!! ROI参数回调被调用，次数: %d，参数数量: %zu !!!", callback_count, parameters.size());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      RCLCPP_ERROR(this->get_logger(), "!!! 处理参数: %s !!!", param.get_name().c_str());

      if (param.get_name() == "roi_width")
      {
        int new_width = param.as_int();
        // 验证新的ROI参数是否会导致超出图像边界
        if (!validate_roi_bounds(roi_.x, roi_.y, new_width, roi_.height))
        {
          RCLCPP_ERROR(this->get_logger(), "ROI宽度参数无效: %d，会导致ROI超出图像边界", new_width);
          RCLCPP_ERROR(this->get_logger(), "图像尺寸: (%d, %d), 当前ROI: (%d, %d, %d, %d)",
                       image_width_, image_height_, roi_.x, roi_.y, roi_.width, roi_.height);
          result.successful = false;
          return result;
        }
        // 更新ROI
        roi_.width = new_width;
        RCLCPP_ERROR(this->get_logger(), "!!! ROI宽度更新为: %d !!!", roi_.width);
        RCLCPP_ERROR(this->get_logger(), "!!! 当前ROI区域: (%d, %d, %d, %d) !!!", roi_.x, roi_.y, roi_.width, roi_.height);

        // 更新检测器ROI设置
        if (detector_)
        {
          detector_->update_roi(roi_, use_roi_);
          RCLCPP_ERROR(this->get_logger(), "!!! 检测器ROI已更新 !!!");
        }
      }
      else if (param.get_name() == "roi_height")
      {
        int new_height = param.as_int();
        // 验证新的ROI参数是否会导致超出图像边界
        if (!validate_roi_bounds(roi_.x, roi_.y, roi_.width, new_height))
        {
          RCLCPP_ERROR(this->get_logger(), "ROI高度参数无效: %d，会导致ROI超出图像边界", new_height);
          RCLCPP_ERROR(this->get_logger(), "图像尺寸: (%d, %d), 当前ROI: (%d, %d, %d, %d)",
                       image_width_, image_height_, roi_.x, roi_.y, roi_.width, roi_.height);
          result.successful = false;
          return result;
        }
        // 更新ROI
        roi_.height = new_height;
        RCLCPP_ERROR(this->get_logger(), "!!! ROI高度更新为: %d !!!", roi_.height);

        // 更新检测器ROI设置
        if (detector_)
        {
          detector_->update_roi(roi_, use_roi_);
          RCLCPP_ERROR(this->get_logger(), "!!! 检测器ROI已更新 !!!");
        }
      }
      else if (param.get_name() == "roi_x")
      {
        int new_x = param.as_int();
        // 验证新的ROI参数是否会导致超出图像边界
        if (!validate_roi_bounds(new_x, roi_.y, roi_.width, roi_.height))
        {
          RCLCPP_ERROR(this->get_logger(), "ROI X坐标参数无效: %d，会导致ROI超出图像边界", new_x);
          RCLCPP_ERROR(this->get_logger(), "图像尺寸: (%d, %d), 当前ROI: (%d, %d, %d, %d)",
                       image_width_, image_height_, roi_.x, roi_.y, roi_.width, roi_.height);
          result.successful = false;
          return result;
        }
        // 更新ROI
        roi_.x = new_x;
        RCLCPP_ERROR(this->get_logger(), "!!! ROI X坐标更新为: %d !!!", roi_.x);

        // 更新检测器ROI设置
        if (detector_)
        {
          detector_->update_roi(roi_, use_roi_);
          RCLCPP_ERROR(this->get_logger(), "!!! 检测器ROI已更新 !!!");
        }
      }
      else if (param.get_name() == "roi_y")
      {
        int new_y = param.as_int();
        // 验证新的ROI参数是否会导致超出图像边界
        if (!validate_roi_bounds(roi_.x, new_y, roi_.width, roi_.height))
        {
          RCLCPP_ERROR(this->get_logger(), "ROI Y坐标参数无效: %d，会导致ROI超出图像边界", new_y);
          RCLCPP_ERROR(this->get_logger(), "图像尺寸: (%d, %d), 当前ROI: (%d, %d, %d, %d)",
                       image_width_, image_height_, roi_.x, roi_.y, roi_.width, roi_.height);
          result.successful = false;
          return result;
        }
        // 更新ROI
        roi_.y = new_y;
        RCLCPP_ERROR(this->get_logger(), "!!! ROI Y坐标更新为: %d !!!", roi_.y);

        // 更新检测器ROI设置
        if (detector_)
        {
          detector_->update_roi(roi_, use_roi_);
          RCLCPP_ERROR(this->get_logger(), "!!! 检测器ROI已更新 !!!");
        }
      }
      else if (param.get_name() == "use_roi")
      {
        bool new_use_roi = param.as_bool();
        // 如果启用ROI，验证当前ROI参数是否有效
        if (new_use_roi && !validate_roi_bounds(roi_.x, roi_.y, roi_.width, roi_.height))
        {
          RCLCPP_ERROR(this->get_logger(), "无法启用ROI，当前ROI参数无效: (%d, %d, %d, %d)",
                       roi_.x, roi_.y, roi_.width, roi_.height);
          RCLCPP_ERROR(this->get_logger(), "图像尺寸: (%d, %d)", image_width_, image_height_);
          result.successful = false;
          return result;
        }
        // 更新ROI使用状态
        use_roi_ = new_use_roi;
        RCLCPP_ERROR(this->get_logger(), "!!! ROI使用状态更新为: %s !!!", use_roi_ ? "启用" : "禁用");

        // 更新检测器ROI设置
        if (detector_)
        {
          detector_->update_roi(roi_, use_roi_);
          RCLCPP_ERROR(this->get_logger(), "!!! 检测器ROI已更新 !!!");
        }
      }
    }

    return result;
  }

  /**
   * @brief 任务回调函数
   * @param task_msg 任务消息
   */
  void YoloDetect::task_callback(const std_msgs::msg::String::SharedPtr task_msg)
  {
    std::string task_mode = task_msg->data;
    if (task_mode == "aim")
    {
      is_aim_task_ = true;
    }
    else
    {
      is_aim_task_ = false;
    }
  }

  /**
   * @brief 图像回调函数
   * @param msg 图像消息
   */
  void YoloDetect::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // 将ROS图像消息转换为OpenCV格式
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat cv_image = cv_ptr->image;

      if (cv_image.empty())
      {
        RCLCPP_WARN(this->get_logger(), "接收到空图像");
        return;
      }

      // 使用YOLOv5进行检测
      auto start_time = std::chrono::high_resolution_clock::now();
      std::list<yolo_detect::Armor> armors = detector_->detect(cv_image, frame_count_);

      rm_armors.clear();
      // 转换为rm_auto_aim::Armor格式
      for (const auto &armor : armors)
      {
        rm_armors.push_back(convert_to_rm_armor(armor));
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

      // 计算帧率 - 使用当前时间减去上一帧时间
      frame_count_++;
      auto current_time = std::chrono::high_resolution_clock::now();

      if (frame_count_ > 1)
      {
        auto fps_duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time_);
        if (fps_duration.count() > 0)
        {
          current_fps_ = 1000000.0 / fps_duration.count();
        }
      }
      last_frame_time_ = current_time;

      if (debug_mode_)
      {
        // 创建检测结果可视化图像
        cv::Mat result_image = draw_detections(cv_image, armors, current_fps_, duration.count());

        // 发布可视化结果图像
        publish_result_image(result_image, msg->header);
      }

      // 如果PnP求解器可用且处于瞄准任务状态，发布装甲板信息
      if (pnp_solver_ != nullptr && is_aim_task_)
      {
        // 初始化标记
        armors_msg_.header = armor_marker_.header = text_marker_.header = msg->header;
        armors_msg_.armors.clear();
        marker_array_.markers.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;

        for (const auto &rm_armor : rm_armors)
        {
          // 使用PnP求解器计算3D位置
          cv::Mat rvec, tvec;
          bool success = pnp_solver_->solvePnP(rm_armor, rvec, tvec);

          if (success)
          {
            yolo_detect::msg::Armor armor_msg;

            // 填充基本信息
            armor_msg.type = rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(rm_armor.type)];
            armor_msg.number = rm_armor.number;

            // 填充位姿信息
            armor_msg.pose.position.x = tvec.at<double>(0);
            armor_msg.pose.position.y = tvec.at<double>(1);
            armor_msg.pose.position.z = tvec.at<double>(2);

            // 将旋转向量转换为四元数
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);

            // rotation matrix to quaternion
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
            armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(rm_armor.center);

            // 填充关键点 - 使用灯条的顶点和底点
            armor_msg.kpts.clear();
            // 添加左灯条的关键点
            geometry_msgs::msg::Point left_top, left_bottom;
            left_top.x = rm_armor.left_light.top.x;
            left_top.y = rm_armor.left_light.top.y;
            left_bottom.x = rm_armor.left_light.bottom.x;
            left_bottom.y = rm_armor.left_light.bottom.y;
            armor_msg.kpts.emplace_back(left_top);
            armor_msg.kpts.emplace_back(left_bottom);

            // 添加右灯条的关键点
            geometry_msgs::msg::Point right_bottom, right_top;
            right_bottom.x = rm_armor.right_light.bottom.x;
            right_bottom.y = rm_armor.right_light.bottom.y;
            right_top.x = rm_armor.right_light.top.x;
            right_top.y = rm_armor.right_light.top.y;
            armor_msg.kpts.emplace_back(right_bottom);
            armor_msg.kpts.emplace_back(right_top);

            armors_msg_.armors.emplace_back(armor_msg);

            // 填充标记
            armor_marker_.id++;
            armor_marker_.scale.y = rm_armor.type == rm_auto_aim::ArmorType::SMALL ? 0.135 : 0.23;
            armor_marker_.pose = armor_msg.pose;
            text_marker_.id++;
            text_marker_.pose.position = armor_msg.pose.position;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = rm_armor.classfication_result;

            marker_array_.markers.emplace_back(armor_marker_);
            marker_array_.markers.emplace_back(text_marker_);

            // 输出详细信息
            RCLCPP_INFO(this->get_logger(),
                        "    PnP结果: 编号=%s, 类型=%s, 位置=(%.3f, %.3f, %.3f), 距离=%.3f",
                        rm_armor.number.c_str(),
                        rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(rm_armor.type)].c_str(),
                        tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                        armor_msg.distance_to_image_center);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "PnP failed!");
          }
        }

        // 发布装甲板信息
        armors_pub_->publish(armors_msg_);

        // 发布标记
        publishMarkers();
      }

      // 输出检测结果
      RCLCPP_INFO(this->get_logger(), "FPS: %.1f, 检测到 %zu 个装甲板, 耗时 %ld ms",
                  current_fps_, armors.size(), duration.count());

      // 详细输出每个装甲板信息
      for (const auto &rm_armor : rm_armors)
      {
        RCLCPP_INFO(this->get_logger(),
                    "  - 转换后: 编号=%s, 类型=%s, 置信度=%.3f, 分类=%s, 位置: (%.1f, %.1f)",
                    rm_armor.number.c_str(),
                    rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(rm_armor.type)].c_str(),
                    rm_armor.confidence,
                    rm_armor.classfication_result.c_str(),
                    rm_armor.center.x, rm_armor.center.y);
      }
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "检测异常: %s", e.what());
    }
  }

  /**
   * @brief 绘制检测结果
   * @param img 原始图像
   * @param armors 检测到的装甲板列表
   * @param fps 当前帧率
   * @param detection_time 检测耗时（毫秒）
   * @return 可视化结果图像
   */
  cv::Mat YoloDetect::draw_detections(const cv::Mat &img, const std::list<yolo_detect::Armor> &armors, double fps, long detection_time)
  {
    cv::Mat detection = img.clone();

    // 在图像左上角显示FPS信息（绿色字体）
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
    cv::putText(detection, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    // 调试信息：显示当前ROI设置
    RCLCPP_DEBUG(this->get_logger(), "draw_detections: ROI区域: (%d, %d, %d, %d), use_roi: %s",
                 roi_.x, roi_.y, roi_.width, roi_.height, use_roi_ ? "true" : "false");

    // 绘制每个装甲板的检测信息
    for (const auto &armor : armors)
    {
      std::string info = std::to_string(armor.confidence).substr(0, 4) + " " +
                         yolo_detect::COLORS[armor.color] + " " + yolo_detect::ARMOR_NAMES[armor.name] + " " +
                         yolo_detect::ARMOR_TYPES[armor.type];

      // 绘制装甲板边界框
      cv::rectangle(detection, armor.box, cv::Scalar(0, 255, 0), 2);
      // 绘制角点
      for (size_t i = 0; i < armor.points.size(); ++i)
      {
        cv::circle(detection, armor.points[i], 3, cv::Scalar(0, 255, 0), -1);

        cv::putText(detection, std::to_string(i), cv::Point(armor.points[i].x, armor.points[i].y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      }

      // 绘制中心点
      cv::circle(detection, armor.center, 5, cv::Scalar(0, 0, 255), -1);

      // 绘制信息文本
      cv::putText(detection, info, cv::Point(armor.box.x, armor.box.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    // 绘制ROI区域
    if (use_roi_)
    {
      cv::Scalar green(0, 255, 0);
      cv::rectangle(detection, roi_, green, 2);

      // 在ROI矩形上显示尺寸信息
      std::string roi_info = "ROI: " + std::to_string(roi_.width) + "x" + std::to_string(roi_.height);
      cv::putText(detection, roi_info, cv::Point(roi_.x, roi_.y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, green, 2);
    }

    return detection;
  }

  /**
   * @brief 发布检测结果图像
   * @param img 检测结果图像
   * @param header 原始图像头信息
   */
  void YoloDetect::publish_result_image(const cv::Mat &img, const std_msgs::msg::Header &header)
  {
    try
    {
      // 将OpenCV图像转换为ROS消息
      cv_bridge::CvImage cv_img;
      cv_img.header = header;
      cv_img.encoding = sensor_msgs::image_encodings::BGR8;
      cv_img.image = img;

      // 发布图像
      result_img_pub_->publish(*cv_img.toImageMsg());
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "发布结果图像异常: %s", e.what());
    }
  }

  /**
   * @brief 将yolo_detect::Armor转换为rm_auto_aim::Armor结构
   * @param source_armor 源yolo_detect::Armor对象
   * @return 转换后的rm_auto_aim::Armor结构
   */
  rm_auto_aim::Armor YoloDetect::convert_to_rm_armor(const yolo_detect::Armor &source_armor)
  {
    rm_auto_aim::Armor target_armor;

    // 1. 设置装甲板中心点
    target_armor.center = source_armor.center;

    // 2. 转换装甲板类型
    if (source_armor.type == yolo_detect::ArmorType::big)
    {
      target_armor.type = rm_auto_aim::ArmorType::LARGE;
    }
    else
    {
      target_armor.type = rm_auto_aim::ArmorType::SMALL;
    }

    // 3. 设置置信度
    target_armor.confidence = source_armor.confidence;

    // 4. 设置数字编号（根据yolo_detect的ArmorName转换为字符串）
    if (source_armor.name >= yolo_detect::ArmorName::one && source_armor.name <= yolo_detect::ArmorName::five)
    {
      // 数字1-5转换为对应字符串
      int number = static_cast<int>(source_armor.name) + 1;
      target_armor.number = std::to_string(number);
    }
    else if (source_armor.name == yolo_detect::ArmorName::sentry)
    {
      target_armor.number = "sentry";
    }
    else if (source_armor.name == yolo_detect::ArmorName::outpost)
    {
      target_armor.number = "outpost";
    }
    else if (source_armor.name == yolo_detect::ArmorName::base)
    {
      target_armor.number = "base";
    }
    else
    {
      target_armor.number = "unknown";
    }

    // 5. 设置分类结果（结合类型和数字信息）
    target_armor.classfication_result = target_armor.number + "_" + rm_auto_aim::ARMOR_TYPE_STR[static_cast<int>(target_armor.type)];

    // 6. 创建虚拟的灯条信息（由于yolo_detect::Armor没有灯条详细信息，根据边界框创建）
    // 根据装甲板的四个顶点估算灯条位置
    if (source_armor.points.size() >= 4)
    {
      // 假设前两个点是左侧灯条，后两个点是右侧灯条
      cv::Point2f left_top = source_armor.points[0];
      cv::Point2f left_bottom = source_armor.points[2];
      cv::Point2f right_top = source_armor.points[1];
      cv::Point2f right_bottom = source_armor.points[3];

      // 计算灯条中心
      cv::Point2f left_center = (left_top + left_bottom) / 2;
      cv::Point2f right_center = (right_top + right_bottom) / 2;

      // 计算灯条长度和倾斜角度
      double left_length = cv::norm(left_top - left_bottom);
      double right_length = cv::norm(right_top - right_bottom);
      float left_angle = std::atan2(left_bottom.y - left_top.y, left_bottom.x - left_top.x);
      float right_angle = std::atan2(right_bottom.y - right_top.y, right_bottom.x - right_top.x);

      // 创建灯条边界框（简化处理）
      cv::Rect left_box(left_center.x - 2, left_center.y - left_length / 2, 4, left_length);
      cv::Rect right_box(right_center.x - 2, right_center.y - right_length / 2, 4, right_length);

      // 创建灯条对象
      target_armor.left_light = rm_auto_aim::Light(left_box, left_top, left_bottom,
                                                   static_cast<int>(left_box.area()), left_angle);
      target_armor.right_light = rm_auto_aim::Light(right_box, right_top, right_bottom,
                                                    static_cast<int>(right_box.area()), right_angle);

      // 设置灯条颜色
      int color = (source_armor.color == yolo_detect::Color::red) ? rm_auto_aim::RED : rm_auto_aim::BLUE;
      target_armor.left_light.color = color;
      target_armor.right_light.color = color;
    }

    // 7. 初始化数字图像（空矩阵，使用不到）
    target_armor.number_img = cv::Mat();

    return target_armor;
  }

  /**
   * @brief 发布标记函数
   */
  void YoloDetect::publishMarkers()
  {
    using Marker = visualization_msgs::msg::Marker;
    armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
  }

} // namespace yolo_detect

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<yolo_detect::YoloDetect>();

  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}