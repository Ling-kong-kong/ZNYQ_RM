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
#include "cv_bridge/cv_bridge.h"
#include "yolo_detect/ros2_armor.hpp"
#include "yolo_detect/yolov5.hpp"
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
    // ROS2 相关成员
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_img_pub_;
    std::unique_ptr<yolo_detect::YOLOV5> detector_;

    // 配置参数
    std::string config_path_;
    std::string image_topic_;
    bool debug_mode_;
    int frame_count_;

    // 帧率计算相关变量
    std::chrono::high_resolution_clock::time_point last_fps_time_;
    double current_fps_ = 0.0;
    std::vector<double> fps_history_;        // FPS历史记录，用于平滑显示
    const size_t max_fps_history_size_ = 10; // 保存最近10帧的FPS

    // ROI参数
    bool use_roi_;
    cv::Rect roi_;
    
    // 图像尺寸
    int image_width_;
    int image_height_;
    
    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

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
    rm_auto_aim::Armor convert_to_rm_armor(const yolo_detect::Armor& source_armor);
  };

} // namespace yolo_detect

#endif // YOLO_DETECT__YOLO_DETECT_NODE_HPP_