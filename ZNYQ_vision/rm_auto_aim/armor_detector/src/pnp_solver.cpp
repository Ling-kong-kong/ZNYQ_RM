// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
/**
 * @brief PnP解算器构造函数
 * @param camera_matrix 相机内参矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 * @param dist_coeffs 相机畸变系数 [k1, k2, p1, p2, k3]
 * @details 初始化PnP解算器，设置小装甲板和大装甲板的3D模型点坐标
 *          装甲板模型坐标系定义：x轴向前，y轴向左，z轴向上
 *          单位：米(m)
 */
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // 装甲板尺寸定义（单位：米）
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;  // 小装甲板半宽度
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0; // 小装甲板半高度
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;  // 大装甲板半宽度
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0; // 大装甲板半高度

  // 定义小装甲板的4个3D模型点（从左下角开始，顺时针方向）
  // 坐标系定义：x向前，y向左，z向上
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));  // 左下
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));   // 左上
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));  // 右上
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z)); // 右下

  // 定义大装甲板的4个3D模型点（从左下角开始，顺时针方向）
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));  // 左下
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));   // 左上
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));  // 右上
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z)); // 右下
}

/**
 * @brief 使用PnP算法求解装甲板的3D位姿
 * @param armor 装甲板对象，包含灯条位置和类型信息
 * @param rvec 输出的旋转向量
 * @param tvec 输出的平移向量
 * @return PnP求解是否成功
 * @details 使用OpenCV的solvePnP函数求解装甲板的3D位姿
 *          输入点为装甲板四个角点的图像坐标（从灯条端点提取）
 *          根据装甲板类型选择对应的大/小装甲板3D模型点
 *          使用IPPE算法，该方法适用于共面点的PnP求解
 */
bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // 提取装甲板四个角点的图像坐标（从灯条端点）
  image_armor_points.emplace_back(armor.left_light.bottom);  // 左灯条底部
  image_armor_points.emplace_back(armor.left_light.top);     // 左灯条顶部
  image_armor_points.emplace_back(armor.right_light.top);    // 右灯条顶部
  image_armor_points.emplace_back(armor.right_light.bottom); // 右灯条底部

  // 根据装甲板类型选择对应的3D模型点
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
  
  // 使用IPPE算法进行PnP求解（适用于共面点的情况）
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

/**
 * @brief 计算图像点到图像中心的距离
 * @param image_point 图像点坐标
 * @return 到图像中心的欧式距离
 * @details 从相机内参矩阵中提取主点坐标(cx, cy)，计算点到主点的距离
 */
float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  // 从相机内参矩阵中提取主点坐标
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  // 计算点到图像中心的欧式距离
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
