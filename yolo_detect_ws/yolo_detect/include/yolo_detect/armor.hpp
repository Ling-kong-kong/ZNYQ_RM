/**
 * @file armor.hpp
 * @brief 装甲板相关数据结构和枚举定义
 * @details 定义了装甲板识别所需的各种数据结构，包括颜色、类型、名称等枚举
 *
 * 主要内容：
 * 1. 装甲板颜色、类型、名称的枚举定义
 * 2. 灯条和装甲板的数据结构
 * 3. 装甲板的空间位置信息
 */

#ifndef YOLO_DETECT__ARMOR_HPP
#define YOLO_DETECT__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace yolo_detect
{
  /**
   * @enum Color
   * @brief 装甲板颜色枚举
   */
  enum Color
  {
    red,        ///< 红色
    blue,       ///< 蓝色
    extinguish, ///< 熄灭
    purple      ///< 紫色（基地）
  };
  const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

  /**
   * @enum ArmorType
   * @brief 装甲板类型枚举
   */
  enum ArmorType
  {
    big,  ///< 大装甲板
    small ///< 小装甲板
  };
  const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

  /**
   * @enum ArmorName
   * @brief 装甲板名称/编号枚举
   */
  enum ArmorName
  {
    one,      ///< 1号装甲板
    two,      ///< 2号装甲板
    three,    ///< 3号装甲板
    four,     ///< 4号装甲板
    five,     ///< 5号装甲板
    sentry,   ///< 哨兵
    outpost,  ///< 前哨站
    base,     ///< 基地
    not_armor ///< 非装甲板
  };
  const std::vector<std::string> ARMOR_NAMES = {"one", "two", "three", "four", "five",
                                                "sentry", "outpost", "base", "not_armor"};

  /**
   * @enum ArmorPriority
   * @brief 装甲板优先级枚举
   */
  enum ArmorPriority
  {
    first = 1, ///< 第一优先级
    second,    ///< 第二优先级
    third,     ///< 第三优先级
    forth,     ///< 第四优先级
    fifth      ///< 第五优先级
  };

  // 装甲板属性配置表（颜色、名称、类型）
  // clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
  // clang-format on

  /**
   * @struct Lightbar
   * @brief 灯条数据结构
   * @details 存储检测到的灯条的各种几何属性
   */
  struct Lightbar
  {
    std::size_t id;                                  ///< 灯条ID
    Color color;                                     ///< 灯条颜色
    cv::Point2f center, top, bottom, top2bottom;     ///< 灯条中心点、顶部点、底部点、顶到底向量
    std::vector<cv::Point2f> points;                 ///< 灯条轮廓点集
    double angle, angle_error, length, width, ratio; ///< 角度、角度误差、长度、宽度、长宽比
    cv::RotatedRect rotated_rect;                    ///< 旋转矩形

    /**
     * @brief 构造函数
     * @param rotated_rect 旋转矩形
     * @param id 灯条ID
     */
    Lightbar(const cv::RotatedRect &rotated_rect, std::size_t id);
    Lightbar() {};
  };

  /**
   * @struct Armor
   * @brief 装甲板数据结构
   * @details 存储装甲板的各种属性，包括几何信息、识别结果和空间位置
   */
  struct Armor
  {
    Color color;                     ///< 装甲板颜色
    Lightbar left, right;            ///< 左右灯条
    cv::Point2f center;              ///< 中心点（不是对角线交点，不能作为实际中心！）
    cv::Point2f center_norm;         ///< 归一化坐标
    std::vector<cv::Point2f> points; ///< 装甲板四个顶点

    double ratio;             ///< 两灯条的中点连线与长灯条的长度之比
    double side_ratio;        ///< 长灯条与短灯条的长度之比
    double rectangular_error; ///< 灯条和中点连线所成夹角与π/2的差值

    ArmorType type;         ///< 装甲板类型
    ArmorName name;         ///< 装甲板名称
    ArmorPriority priority; ///< 装甲板优先级
    int class_id;           ///< 分类ID
    cv::Rect box;           ///< 边界框
    cv::Mat pattern;        ///< 图案
    double confidence;      ///< 置信度
    bool duplicated;        ///< 是否重复

    // 空间位置信息
    Eigen::Vector3d xyz_in_gimbal; ///< 在云台坐标系下的位置 (m)
    Eigen::Vector3d xyz_in_world;  ///< 在世界坐标系下的位置 (m)
    Eigen::Vector3d ypr_in_gimbal; ///< 在云台坐标系下的欧拉角 (rad)
    Eigen::Vector3d ypr_in_world;  ///< 在世界坐标系下的欧拉角 (rad)
    Eigen::Vector3d ypd_in_world;  ///< 在世界坐标系下的球坐标

    double yaw_raw; ///< 原始偏航角 (rad)

    Armor(
        int color_id, int num_id, float confidence, const cv::Rect &box,
        std::vector<cv::Point2f> armor_keypoints);
    Armor(
        int color_id, int num_id, float confidence, const cv::Rect &box,
        std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
  };

} // namespace yolo_detect

#endif // YOLO_DETECT__ARMOR_HPP