// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
/**
 * @brief 扩展卡尔曼滤波器构造函数
 * @param f 非线性状态转移函数
 * @param h 非线性观测函数
 * @param j_f 状态转移函数的雅可比矩阵
 * @param j_h 观测函数的雅可比矩阵
 * @param u_q 过程噪声协方差矩阵更新函数
 * @param u_r 观测噪声协方差矩阵更新函数
 * @param P0 初始状态协方差矩阵
 * @details 初始化扩展卡尔曼滤波器的所有参数和矩阵
 *          
 *          扩展卡尔曼滤波器的基本原理：
 *          1. 对非线性函数进行泰勒展开，保留一阶项，得到线性化模型
 *          2. 使用线性化的卡尔曼滤波器进行状态估计
 *          3. 在每一步更新中重新计算雅可比矩阵
 *          
 *          在装甲板跟踪中的应用：
 *          - 状态转移函数描述了目标的运动模型（匀速运动）
 *          - 观测函数描述了从机器人中心到装甲板的几何关系
 *          - 过程噪声考虑了目标运动的不可预测性
 *          - 观测噪声考虑了检测误差
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
  // 初始化所有成员变量
  // P_post: 初始误差协方差矩阵，表示对初始状态的不确定性
  // n: 状态向量的维度
  // I: 单位矩阵，用于协方差矩阵更新
  // x_pri: 先验状态向量（预测状态）
  // x_post: 后验状态向量（更新后状态）
}

/**
 * @brief 设置滤波器初始状态
 * @param x0 初始状态向量
 * @details 在开始跟踪前设置初始状态向量，通常在检测到新目标时调用
 */
void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

/**
 * @brief 预测步骤
 * @return 预测的状态向量
 * @details 扩展卡尔曼滤波器的预测步骤：
 *          1. 计算状态转移函数的雅可比矩阵
 *          2. 更新过程噪声协方差矩阵
 *          3. 通过非线性状态转移函数预测状态
 *          4. 更新预测状态协方差矩阵
 *          5. 将预测状态作为当前状态（处理无测量情况）
 *          
 *          预测方程：
 *          x_{k|k-1} = f(x_{k-1|k-1})
 *          P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
 *          
 *          其中：
 *          - x_{k|k-1}: k时刻的先验状态估计
 *          - f(): 非线性状态转移函数
 *          - P_{k|k-1}: k时刻的先验误差协方差
 *          - F: 状态转移函数的雅可比矩阵
 *          - Q: 过程噪声协方差矩阵
 */
Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  // 计算状态转移函数在当前状态点的雅可比矩阵和过程噪声协方差矩阵
  F = jacobian_f(x_post), Q = update_Q(x_post);

  // 通过非线性状态转移函数预测下一时刻的状态
  x_pri = f(x_post);
  
  // 更新预测状态的协方差矩阵
  // P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
  P_pri = F * P_post * F.transpose() + Q;

  // 处理在下次预测前没有测量的情况
  // 将预测状态作为当前状态，确保状态连续性
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

/**
 * @brief 更新步骤
 * @param z 观测向量
 * @return 更新后的状态向量
 * @details 扩展卡尔曼滤波器的更新步骤：
 *          1. 计算观测函数的雅可比矩阵
 *          2. 更新观测噪声协方差矩阵
 *          3. 计算卡尔曼增益
 *          4. 更新状态估计
 *          5. 更新状态协方差矩阵
 *          
 *          更新方程：
 *          K = P_{k|k-1} * H^T * (H * P_{k|k-1} * H^T + R)^{-1}
 *          x_{k|k} = x_{k|k-1} + K * (z_k - h(x_{k|k-1}))
 *          P_{k|k} = (I - K * H) * P_{k|k-1}
 *          
 *          其中：
 *          - K: 卡尔曼增益，表示对观测的信任程度
 *          - H: 观测函数的雅可比矩阵
 *          - R: 观测噪声协方差矩阵
 *          - z_k: k时刻的观测值
 *          - h(): 非线性观测函数
 *          - x_{k|k}: k时刻的后验状态估计
 *          - P_{k|k}: k时刻的后验误差协方差
 */
Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  // 计算观测函数在预测状态点的雅可比矩阵和观测噪声协方差矩阵
  H = jacobian_h(x_pri), R = update_R(z);

  // 计算卡尔曼增益
  // 卡尔曼增益平衡了预测和观测的权重
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  
  // 更新状态估计
  // 新状态 = 预测状态 + 卡尔曼增益 * (观测值 - 预测观测值)
  x_post = x_pri + K * (z - h(x_pri));
  
  // 更新状态协方差矩阵
  // 新协方差 = (I - 卡尔曼增益 * 观测矩阵) * 预测协方差
  P_post = (I - K * H) * P_pri;

  return x_post;
}

}  // namespace rm_auto_aim
