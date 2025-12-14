/**
 * @file armor.cpp
 * @brief 装甲板和灯条类实现文件
 * @details 实现装甲板和灯条的几何计算、构造函数等功能
 *
 * 主要功能：
 * 1. 灯条的几何特征计算（长度、宽度、角度等）
 * 2. 装甲板的多种构造方式（传统检测、神经网络、YOLO等）
 * 3. 装甲板的几何属性计算（长宽比、矩形误差等）
 */

#include "yolo_detect/armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace yolo_detect
{
  /**
   * @brief YOLOv5装甲板构造函数（无ROI）
   * @param color_id 颜色ID（0=蓝色，1=红色，其他=熄灭）
   * @param num_id 数字ID（0=哨兵，1=大装甲板，其他=小装甲板）
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板四个角点坐标
   * @details
   * 这个构造函数专门用于处理YOLOv5的输出结果，YOLOv5分别输出颜色和数字的类别。
   * 主要特点：
   * 1. 独立的颜色和数字识别：color_id表示颜色，num_id表示数字
   * 2. 直接根据ID计算装甲板属性，不依赖预定义数组
   * 3. 特殊处理哨兵（num_id=0）和大装甲板（num_id=1）
   * 4. 支持熄灭状态（color_id非0或1时）
   */
  Armor::Armor(
      int color_id, int num_id, float confidence, const cv::Rect &box,
      std::vector<cv::Point2f> armor_keypoints)
      : points(armor_keypoints), box(box), confidence(confidence)
  {
    // 计算装甲板中心点（四个角点的平均值）
    center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;

    // 计算左右两侧的宽度
    auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);  // 左侧宽度
    auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]); // 右侧宽度
    auto max_width = std::max(left_width, right_width);                   // 最大宽度

    // 计算上下两侧的长度
    auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);    // 上侧长度
    auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]); // 下侧长度
    auto max_length = std::max(top_length, bottom_length);                  // 最大长度

    // 计算左右中心点
    auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
    auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
    auto left2right = right_center - left_center; // 左右中心点向量

    // 计算装甲板滚转角
    auto roll = std::atan2(left2right.y, left2right.x);

    // 计算左右灯条与装甲板平面的偏差（矩形误差）
    auto left_rectangular_error = std::abs(
        std::atan2(
            (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
        roll - CV_PI / 2);
    auto right_rectangular_error = std::abs(
        std::atan2(
            (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
        roll - CV_PI / 2);
    rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

    // 计算长宽比
    ratio = max_length / max_width;

    // 根据颜色ID设置颜色（支持蓝色、红色和熄灭状态）
    color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red
                                                        : Color::extinguish;

    // 根据数字ID设置装甲板名称
    // 0=哨兵，1=大装甲板（英雄），2-5=小装甲板（数字3-6），>5=特殊编号
    name = num_id == 0  ? ArmorName::sentry
           : num_id > 5 ? ArmorName(num_id)
                        : ArmorName(num_id - 1); // TODO: 考虑Bb（平衡步兵）

    // 根据数字ID设置装甲板类型（1=大装甲板，其他=小装甲板）
    type = num_id == 1 ? ArmorType::big : ArmorType::small;
  }

  /**
   * @brief YOLOv5装甲板构造函数（带ROI偏移）
   * @param color_id 颜色ID（0=蓝色，1=红色，其他=熄灭）
   * @param num_id 数字ID（0=哨兵，1=大装甲板，其他=小装甲板）
   * @param confidence 置信度
   * @param box 边界框
   * @param armor_keypoints 装甲板四个角点坐标（相对于ROI区域）
   * @param offset ROI区域相对于全图像的偏移量
   * @details
   * 这个构造函数是YOLOv5构造函数的ROI版本，结合了YOLOv5的独立颜色/数字识别和ROI坐标转换功能。
   * 主要特点：
   * 1. 支持ROI区域检测，提高检测效率
   * 2. 将ROI坐标转换为全图像坐标
   * 3. 保持YOLOv5的独立颜色和数字识别逻辑
   * 4. 适用于需要在特定区域进行高效检测的场景
   */
  Armor::Armor(
      int color_id, int num_id, float confidence, const cv::Rect &box,
      std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
      : points(armor_keypoints), confidence(confidence)
  {
    // 将ROI区域内的角点坐标转换为全图像坐标（加上偏移量）
    std::transform(
        armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
        [&offset](const cv::Point2f &point)
        { return point + offset; });

    // 同时更新points数组中的坐标
    std::transform(
        points.begin(), points.end(), points.begin(),
        [&offset](const cv::Point2f &point)
        { return point + offset; });

    // 将边界框坐标也转换到全局坐标系
    this->box = cv::Rect(
        box.x + static_cast<int>(offset.x),
        box.y + static_cast<int>(offset.y),
        box.width,
        box.height
    );

    // 计算装甲板中心点（四个角点的平均值）
    center = (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;

    // 计算左右两侧的宽度
    auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);  // 左侧宽度
    auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]); // 右侧宽度
    auto max_width = std::max(left_width, right_width);                   // 最大宽度

    // 计算上下两侧的长度
    auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);    // 上侧长度
    auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]); // 下侧长度
    auto max_length = std::max(top_length, bottom_length);                  // 最大长度

    // 计算左右中心点
    auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
    auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
    auto left2right = right_center - left_center; // 左右中心点向量

    // 计算装甲板滚转角
    auto roll = std::atan2(left2right.y, left2right.x);

    // 计算左右灯条与装甲板平面的偏差（矩形误差）
    auto left_rectangular_error = std::abs(
        std::atan2(
            (armor_keypoints[3] - armor_keypoints[0]).y, (armor_keypoints[3] - armor_keypoints[0]).x) -
        roll - CV_PI / 2);
    auto right_rectangular_error = std::abs(
        std::atan2(
            (armor_keypoints[2] - armor_keypoints[1]).y, (armor_keypoints[2] - armor_keypoints[1]).x) -
        roll - CV_PI / 2);
    rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

    // 计算长宽比
    ratio = max_length / max_width;

    // 根据颜色ID设置颜色（支持蓝色、红色和熄灭状态）
    color = color_id == 0 ? Color::blue : color_id == 1 ? Color::red
                                                        : Color::extinguish;

    // 根据数字ID设置装甲板名称
    // 0=哨兵，1=大装甲板（英雄），2-5=小装甲板（数字3-6），>5=特殊编号
    name = num_id == 0 ? ArmorName::sentry : num_id > 5 ? ArmorName(num_id)
                                                        : ArmorName(num_id - 1);

    // 根据数字ID设置装甲板类型（1=大装甲板，其他=小装甲板）
    type = num_id == 1 ? ArmorType::big : ArmorType::small;
  }

} // namespace yolo_detect