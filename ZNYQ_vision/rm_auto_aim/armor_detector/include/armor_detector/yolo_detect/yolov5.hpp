/**
 * @file yolov5.hpp
 * @brief YOLOv5检测器模块头文件
 * @details 该模块实现了基于YOLOv5的装甲板检测功能
 *
 * 主要功能：
 * 1. YOLOv5模型推理
 * 2. 检测结果后处理
 * 3. 与传统方法结合的混合检测
 * 4. ROI区域检测
 */

#ifndef YOLO_DETECT__YOLOV5_HPP
#define YOLO_DETECT__YOLOV5_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "armor_detector/yolo_detect/armor.hpp"

namespace armor_detector
{
namespace yolo_detect
{
  /**
   * @class YOLOV5
   * @brief YOLOv5检测器类
   * @details 继承自YOLOBase，实现YOLOv5特定的检测逻辑
   */
  class YOLOV5
  {
  public:
    /**
     * @brief 构造函数
     * @param config_path 配置文件路径
     * @param debug 是否开启调试模式
     * @param use_roi 是否使用ROI区域
     * @param roi ROI区域参数
     */
    YOLOV5(const std::string &config_path, bool debug, bool use_roi, const cv::Rect &roi);

    /**
     * @brief 检测装甲板
     * @param bgr_img BGR图像
     * @param frame_count 帧计数
     * @return 检测到的装甲板列表
     */
    std::list<Armor> detect(const cv::Mat &bgr_img, int frame_count);

    /**
     * @brief 后处理接口函数
     * @param scale 缩放比例
     * @param output 神经网络输出
     * @param bgr_img 输入图像
     * @param frame_count 帧计数
     * @return 处理后的装甲板列表
     */
    std::list<Armor> postprocess(
        double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count);

    /**
     * @brief 更新ROI设置
     * @param roi 新的ROI区域
     * @param use_roi 是否使用ROI
     */
    void update_roi(const cv::Rect &roi, bool use_roi);

  private:
    std::string device_, model_path_;    ///< 推理设备和模型路径
    std::string debug_path_;           ///< 调试路径
    bool debug_, use_roi_;               ///< 调试标志、ROI标志
    // bool debug_, use_roi_, use_traditional_;  ///< 调试标志、ROI标志、传统方法标志

    // 检测参数
    const int class_num_ = 13;          ///< 类别数量
    const float nms_threshold_ = 0.3;   ///< NMS阈值
    const float score_threshold_ = 0.7; ///< 分数阈值
    double min_confidence_;             ///< 最小置信度
    // double min_confidence_, binary_threshold_;  ///< 最小置信度和二值化阈值

    ov::Core core_;                    ///< OpenVINO推理核心
    ov::CompiledModel compiled_model_; ///< 编译后的模型

    cv::Rect roi_;       ///< 感兴趣区域
    cv::Point2f offset_; ///< 偏移量
    cv::Mat tmp_img_;    ///< 临时图像

    // Detector detector_;                   ///< 传统检测器 (暂时禁用)
    // friend class MultiThreadDetector;     ///< 友元类 (暂时禁用)

    /**
     * @brief 检查装甲板名称
     * @param armor 装甲板对象
     * @return 是否通过名称检查
     */
    bool check_name(const Armor &armor) const;

    /**
     * @brief 检查装甲板类型
     * @param armor 装甲板对象
     * @return 是否通过类型检查
     */
    bool check_type(const Armor &armor) const;

    /**
     * @brief 获取归一化中心点
     * @param bgr_img BGR图像
     * @param center 中心点
     * @return 归一化中心点
     */
    cv::Point2f get_center_norm(const cv::Mat &bgr_img, const cv::Point2f &center) const;

    /**
     * @brief 解析网络输出
     * @param scale 缩放比例
     * @param output 网络输出
     * @param bgr_img BGR图像
     * @param frame_count 帧计数
     * @return 装甲板列表
     */
    std::list<Armor> parse(double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count);

    

    /**
     * @brief 绘制检测结果
     * @param img 图像
     * @param armors 装甲板列表
     * @param frame_count 帧计数
     */
    void draw_detections(const cv::Mat &img, const std::list<Armor> &armors, int frame_count) const;

    /**
     * @brief Sigmoid激活函数
     * @param x 输入值
     * @return 激活后的值
     */
    double sigmoid(double x);
  };

} // namespace yolo_detect
} // namespace armor_detector

#endif // YOLO_DETECT__YOLOV5_HPP