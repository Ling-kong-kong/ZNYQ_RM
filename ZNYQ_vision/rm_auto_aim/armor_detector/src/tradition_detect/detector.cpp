// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/tradition_detect/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace armor_detector
{
namespace tradition_detect
{
/**
 * @brief 装甲板检测器构造函数
 * @param bin_thres 二值化阈值
 * @param color 检测颜色（0=红色，1=蓝色）
 * @param l 灯条参数结构体
 * @param a 装甲板参数结构体
 * @details 初始化装甲板检测器，设置二值化阈值、检测颜色和灯条/装甲板的筛选参数
 */
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

/**
 * @brief 装甲板检测主函数
 * @param input 输入的RGB图像
 * @return 检测到的装甲板列表
 * @details 完整的装甲板检测流程：
 *          1. 图像预处理（灰度化、二值化）
 *          2. 灯条检测
 *          3. 灯条配对形成装甲板候选
 *          4. 数字识别（如果有装甲板）
 *          5. 返回最终的装甲板列表
 */
std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  // 1. 图像预处理：将RGB图像转换为二值图像
  binary_img = preprocessImage(input);
  // 2. 检测图像中的灯条
  lights_ = findLights(input, binary_img);
  // 3. 将灯条配对形成装甲板候选
  armors_ = matchLights(lights_);

  // 4. 如果检测到装甲板，进行数字识别和分类
  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

/**
 * @brief 图像预处理函数
 * @param rgb_img 输入的RGB图像
 * @return 二值化后的图像
 * @details 将RGB图像转换为灰度图，然后使用固定阈值进行二值化处理
 *          二值化阈值由binary_thres参数控制
 */
cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::Mat gray_img;
  // 将RGB图像转换为灰度图像
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  // 使用固定阈值进行二值化处理
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

/**
 * @brief 在二值化图像中检测灯条
 * @param rgb_img 原�始RGB图像（用于颜色判断）
 * @param binary_img 二值化图像
 * @return 检测到的灯条列表
 * @details 灯条检测流程：
 *          1. 使用findContours查找二值图像中的所有轮廓
 *          2. 对每个轮廓进行几何分析，计算外接矩形和倾斜角度
 *          3. 使用最小二乘法拟合直线，提高角度计算精度
 *          4. 根据几何参数和填充比例筛选有效灯条
 *          5. 通过RGB值判断灯条颜色（红色或蓝色）
 */
std::vector<Light> Detector::findLights(const cv::Mat & rgb_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  // 查找二值图像中的所有轮廓
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  // 遍历每个轮廓，尝试识别为灯条
  for (const auto & contour : contours) {
    // 忽略过小的轮廓（可能为噪声）
    if (contour.size() < 5) continue;

    // 计算轮廓的外接矩形和旋转矩形
    auto b_rect = cv::boundingRect(contour);
    auto r_rect = cv::minAreaRect(contour);
    
    // 创建掩码并计算轮廓内的点
    cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
    std::vector<cv::Point> mask_contour;
    for (const auto & p : contour) {
      mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
    }
    cv::fillPoly(mask, {mask_contour}, 255);
    std::vector<cv::point> points;
    cv::findNonZero(mask, points);
    
    // 计算轮廓点占旋转矩形面积的比例（填充比例）
    bool is_fill_rotated_rect =
      points.size() / (r_rect.size.width * r_rect.size.height) > l.min_fill_ratio;
    
    // 使用最小二乘法拟合直线，提高角度计算精度
    cv::Vec4f return_param;
    cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f top, bottom;
    double angle_k;
    
    // 处理垂直或水平的特殊情况
    if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
      top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
      bottom = cv::Point2f(b_rect.x + b_rect / 2, b_rect.y + b_rect.height);
      angle_k = 0;
    } else {
      // 计算直线斜率和截距
      auto k = return_param[1] / return_param[0];
      auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
      // 计算直线与矩形边界的交点作为灯条端点
      top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
      bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
      // 计算倾斜角度（转换为度数）
      angle_k = std::atan(k) / CV_PI * 180 - 90;
      if (angle_k > 90) {
        angle_k = 180 - angle_k;
      }
    }
    
    // 创建灯条对象
    auto light = Light(b_rect, top, bottom, points.size(), angle_k);

    // 验证灯条是否符合几何条件和填充比例要求
    if (isLight(light) && is_fill_rotated_rect) {
      auto rect = light;
      // 确保矩形在图像边界内
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rgb_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rgb_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rgb_img(rect);
        // 遍历ROI内的每个像素点
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            // 检查点是否在轮廓内
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // 累加红色和蓝色通道的像素值
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // 根据红色和蓝色像素值判断灯条颜色
        light.color = sum_r > sum_b ? RED : BLUE;
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

/**
 * @brief 验证候选灯条是否符合灯条特征
 * @param light 待验证的灯条
 * @return 是否为有效灯条
 * @details 检查灯条的长宽比和倾斜角度是否在预设范围内
 *          同时填充调试信息用于可视化
 */
bool Detector::isLight(const Light & light)
{
  // 计算灯条的长宽比（短边/长边）
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  // 检查灯条的倾斜角度是否在允许范围内
  bool angle_ok = light.tilt_angle < l.max_angle;

  // 综合判断是否为有效灯条
  bool is_light = ratio_ok && angle_ok;

  // 填充调试信息
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

/**
 * @brief 将灯条配对形成装甲板候选
 * @param lights 检测到的灯条列表
 * @return 检测到的装甲板候选列表
 * @details 装甲板配对流程：
 *          1. 遍历所有灯条对组合
 *          2. 确保两灯条颜色与目标颜色一致
          3. 检查是否有其他灯条在两灯条形成的矩形内
 *          4. 验证几何约束（长度比、中心距离、角度）
 *          5. 创建装甲板对象并添加到结果列表
 */
std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // 遍历所有灯条对组合
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      // 只处理与目标颜色相同的灯条对
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      // 检查是否有其他灯条在两灯条形成的矩形内
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      // 验证这对灯条是否可以形成有效装甲板
      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        // 创建装甲板对象
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

/**
 * @brief 检查是否有其他灯条在两灯条形成的矩形内
 * @param light_1 第一个灯条
 * @param light_2 第二个灯条
 * @param lights 所有灯条列表
 * @return 是否存在其他灯条在矩形内
 * @details 防止将三个灯条误判为一个装甲板
 */
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  // 计算两灯条形成的矩形区域
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  // 检查是否有其他灯条在这个矩形内
  for (const auto & test_light : lights) {
    // 跳过自身灯条
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    // 检查其他灯条是否在矩形边界内
    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

/**
 * @brief 判断两个灯条是否能形成有效装甲板
 * @param light_1 第一个灯条
 * @param light_2 第二个灯条
 * @return 装甲板类型（小装甲板、大装甲板或无效）
 * @details 装甲板判断标准：
 *          1. 两灯条长度比不能过小（防止尺寸不匹配）
 *          2. 灯条中心距离在合理范围内（区分大小装甲板）
 *          3. 灯条中心连线角度不能过大（防止倾斜过大）
 */
ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // 计算两灯条长度比（短边/长边）
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // 计算灯条中心距离（相对于平均灯条长度的比例）
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // 计算灯条中心连线的角度
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // 判断装甲板类型
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // 填充调试信息
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

/**
 * @brief 获取所有装甲板的数字图像
 * @return 垂直拼接的数字图像
 * @details 如果没有检测到装甲板，返回空白图像
 *          否则将所有装甲板的数字图像垂直拼接
 */
cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    // 没有检测到装甲板时返回空白图像
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());

    // 收集所有装甲板的数字图像
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    // 垂直拼接所有数字图像
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

/**
 * @brief 在图像上绘制检测结果
 * @param img 要绘制的图像
 * @details 绘制内容包括：
 *          1. 灯条检测结果（绿色为有效，红色为无效）
 *          2. 装甲板检测结果（绿色为有效，红色为无效）
 *          3. 灯条和装甲板的几何参数
 *          4. 灯条连线（红色/黄色表示颜色）
 *          5. 装甲板轮廓（绿色）
 */
void Detector::drawResults(cv::Mat & img)
{
  // 绘制灯条检测结果
  for (const auto & light : debug_lights.data) {
    // 根据是否为有效灯条选择颜色
    auto color = light.is_light ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    // 显示灯条长宽比
    cv::putText(
      img, std::to_string(light.ratio), cv::Point(light.center_x, light.center_y - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
  }

  // 绘制装甲板检测结果
  for (const auto & armor : debug_armors.data) {
    // 根据是否为有效装甲板选择颜色
    auto color = armor.is_armor ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    // 显示灯条长度比
    cv::putText(
      img, std::to_string(armor.light_ratio), cv::Point(armor.center_x, armor.center_y - 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
    // 显示中心距离
    cv::putText(
      img, std::to_string(armor.center_distance), cv::Point(armor.center_x, armor.center_y + 10),
      cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
    // 显示角度
    cv::putText(
      img, std::to_string(armor.angle), cv::Point(armor.center_x, armor.center_y + 30),
      cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
  }

  // 绘制灯条
  for (const auto & light : lights_) {
    // 根据灯条颜色选择绘制颜色
    auto color = light.color == RED ? cv::Scalar(255, 0, 0) : cv::Scalar(255, 255, 0);
    // 绘制灯条中心线
    cv::line(img, light.top, light.bottom, color, 2);
  }

  // 绘制装甲板轮廓
  for (const auto & armor : armors_) {
    // 绘制装甲板的X形轮廓
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }
}

}  // namespace tradition_detect
}  // namespace armor_detector
