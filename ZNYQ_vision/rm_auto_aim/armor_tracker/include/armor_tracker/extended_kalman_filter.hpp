// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

/**
 * @brief 扩展卡尔曼滤波器类
 * @details 扩展卡尔曼滤波器(EKF)是卡尔曼滤波器在非线性系统中的应用。
 *          它通过线性化非线性函数来处理非线性状态估计问题。
 *          
 *          在RoboMaster自瞄系统中，EKF用于：
 *          1. 预测装甲板下一时刻的位置和速度
 *          2. 融合观测数据更新目标状态估计
 *          3. 处理目标旋转时的装甲板跳变问题
 *          
 *          状态向量定义：
 *          x = [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]^T
 *          其中：
 *          - (xc, yc, za): 机器人中心在惯性系中的位置
 *          - (v_xc, v_yc, v_za): 机器人中心的速度
 *          - yaw: 机器人的偏航角
 *          - v_yaw: 机器人的偏航角速度
 *          - r: 机器人中心到装甲板的距离
 *          
 *          观测向量定义：
 *          z = [xa, ya, za, yaw]^T
 *          其中：
 *          - (xa, ya, za): 装甲板在惯性系中的位置
 *          - yaw: 装甲板的偏航角
 */
class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  // 函数类型定义
  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  /**
   * @brief 扩展卡尔曼滤波器构造函数
   * @param f 非线性状态转移函数，用于状态预测
   * @param h 非线性观测函数，用于计算观测值
   * @param j_f 状态转移函数的雅可比矩阵函数
   * @param j_h 观测函数的雅可比矩阵函数
   * @param u_q 过程噪声协方差矩阵更新函数
   * @param u_r 观测噪声协方差矩阵更新函数
   * @param P0 初始状态协方差矩阵
   * @details 初始化EKF的所有参数和函数对象
   */
  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  /**
   * @brief 设置滤波器初始状态
   * @param x0 初始状态向量
   * @details 在开始跟踪前设置初始状态向量
   */
  void setState(const Eigen::VectorXd & x0);

  /**
   * @brief 执行预测步骤
   * @return 预测的状态向量
   * @details 使用状态转移函数预测下一时刻的状态：
   *          1. 计算状态转移函数的雅可比矩阵
   *          2. 更新过程噪声协方差矩阵
   *          3. 通过非线性状态转移函数预测状态
   *          4. 更新预测状态协方差矩阵
   */
  Eigen::MatrixXd predict();

  /**
   * @brief 执行更新步骤
   * @param z 观测向量
   * @return 更新后的状态向量
   * @details 使用观测数据更新状态估计：
   *          1. 计算观测函数的雅可比矩阵
   *          2. 更新观测噪声协方差矩阵
   *          3. 计算卡尔曼增益
   *          4. 更新状态估计
   *          5. 更新状态协方差矩阵
   */

private:
  // 非线性状态转移函数，用于状态预测
  VecVecFunc f;
  // 非线性观测函数，用于计算观测值
  VecVecFunc h;
  // 状态转移函数的雅可比矩阵函数
  VecMatFunc jacobian_f;
  // 状态转移函数的雅可比矩阵
  Eigen::MatrixXd F;
  // 观测函数的雅可比矩阵函数
  VecMatFunc jacobian_h;
  // 观测函数的雅可比矩阵
  Eigen::MatrixXd H;
  // 过程噪声协方差矩阵更新函数
  VecMatFunc update_Q;
  // 过程噪声协方差矩阵
  Eigen::MatrixXd Q;
  // 观测噪声协方差矩阵更新函数
  VecMatFunc update_R;
  // 观测噪声协方差矩阵
  Eigen::MatrixXd R;

  // 先验误差估计协方差矩阵（预测误差）
  Eigen::MatrixXd P_pri;
  // 后验误差估计协方差矩阵（更新后误差）
  Eigen::MatrixXd P_post;

  // 卡尔曼增益，用于平衡预测和观测的权重
  Eigen::MatrixXd K;

  // 系统状态向量的维度
  int n;

  // N×N单位矩阵
  Eigen::MatrixXd I;

  // 先验状态向量（预测状态）
  Eigen::VectorXd x_pri;
  // 后验状态向量（更新后状态）
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
