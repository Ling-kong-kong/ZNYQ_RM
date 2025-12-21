/**
 * @file yolo_detect_node.hpp
 * @brief YOLO检测节点头文件
 * @details 订阅海康相机图像话题，使用YOLOv5+OpenVINO进行装甲板检测
 */

#ifndef YOLO_DETECT__YOLO_DETECT_NODE_HPP_
#define YOLO_DETECT__YOLO_DETECT_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// 包含生成的消息头文件
#include "yolo_detect/msg/armor.hpp"
#include "yolo_detect/msg/armors.hpp"
#include "cv_bridge/cv_bridge.h"
#include "yolo_detect/ros2_armor.hpp"
#include "yolo_detect/yolov5.hpp"
#include "yolo_detect/pnp_solver.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace yolo_detect
{

  /**
   * @class YoloDetect
   * @brief YOLO检测节点类
   * @details 继承自 rclcpp::Node，实现YOLO检测功能
   */
  class YoloDetect : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数
     */
    YoloDetect();

  private:
  private:
    // 图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // 结果图像发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;

    // yolo检测初始化器
    std::unique_ptr<yolo_detect::YOLOV5> detector_;

    // PnP求解
    std::unique_ptr<rm_auto_aim::PnPSolver> pnp_solver_;

    // 相机信息及订阅发布器
    cv::Point2f cam_center_;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    // 任务状态及任务订阅器
    bool is_aim_task_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;

    // 可视化标记及可视化标记发布器
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // 装甲板信息发布器
    yolo_detect::msg::Armors armors_msg_;
    rclcpp::Publisher<yolo_detect::msg::Armors>::SharedPtr armors_pub_;

    // rm_auto_aim::Armor格式
    std::list<rm_auto_aim::Armor> rm_armors;

    // 配置参数
    std::string config_path_;
    std::string image_topic_;
    bool debug_mode_;
    int frame_count_;

    // 帧率计算相关变量
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    double current_fps_ = 0.0;

    // ROI参数
    bool use_roi_;
    cv::Rect roi_;

    // 图像尺寸
    int image_width_;
    int image_height_;

    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    /**
     * @brief 任务回调函数
     * @param task_msg 任务消息
     */
    void task_callback(const std_msgs::msg::String::SharedPtr task_msg);

    /**
     * @brief 图像回调函数
     * @param msg 图像消息
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 绘制检测结果
     * @param img 原始图像
     * @param armors 检测到的装甲板列表
     * @param fps 当前帧率
     * @param detection_time 检测耗时（毫秒）
     * @return 可视化结果图像
     */
    cv::Mat draw_detections(const cv::Mat &img, const std::list<yolo_detect::Armor> &armors, double fps, long detection_time);

    /**
     * @brief 发布检测结果图像
     * @param img 检测结果图像
     * @param header 原始图像头信息
     */
    void publish_result_image(const cv::Mat &img, const std_msgs::msg::Header &header);

    /**
     * @brief 验证ROI参数是否在图像边界内
     * @param x ROI X坐标
     * @param y ROI Y坐标
     * @param width ROI宽度
     * @param height ROI高度
     * @return 是否在边界内
     */
    bool validate_roi_bounds(int x, int y, int width, int height);

    /**
     * @brief 初始化YOLO相关参数和检测器
     * @return 初始化是否成功
     */
    bool initYolo();

    /**
     * @brief 参数回调函数
     * @param parameters 参数列表
     * @return 设置结果
     */
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    /**
     * @brief ROI参数回调处理函数
     * @param parameters 参数列表
     * @return 设置结果
     */
    rcl_interfaces::msg::SetParametersResult roi_parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters);

    /**
     * @brief 将yolo_detect::Armor转换为rm_auto_aim::Armor结构
     * @param source_armor 源yolo_detect::Armor对象
     * @return 转换后的rm_auto_aim::Armor结构
     */
    rm_auto_aim::Armor convert_to_rm_armor(const yolo_detect::Armor &source_armor);

    /**
     * @brief 发布标记函数
     */
    void publishMarkers();
  };

} // namespace yolo_detect

#endif // YOLO_DETECT__YOLO_DETECT_NODE_HPP_