// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/tradition_detect/armor.hpp"
#include "armor_detector/tradition_detect/number_classifier.hpp"

namespace armor_detector
{
namespace tradition_detect
{
/**
 * @brief 数字分类器构造函数
 * @param model_path ONNX模型文件路径
 * @param label_path 标签文件路径
 * @param thre 分类置信度阈值
 * @param ignore_classes 需要忽略的类别列表
 * @details 初始化神经网络模型和加载类别标签
 */
NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  // 从ONNX文件加载神经网络模型
  net_ = cv::dnn::readNetFromONNX(model_path);

  // 从标签文件加载类别名称
  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

/**
 * @brief 从装甲板图像中提取数字区域
 * @param src 原始图像
 * @param armors 装甲板列表
 * @details 通过透视变换将装甲板区域校正为标准尺寸，然后提取数字区域
 */
void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // 灯条在图像中的长度
  const int light_length = 12;
  // 透视变换后的图像高度
  const int warp_height = 28;
  // 小装甲板宽度
  const int small_armor_width = 32;
  // 大装甲板宽度
  const int large_armor_width = 54;
  // 数字区域ROI大小
  const cv::Size roi_size(20, 28);

  // 处理每个装甲板
  for (auto & armor : armors) {
    // 定义透视变换的四个顶点（装甲板的四个角点）
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    // 计算灯条在变换后图像中的位置
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    // 根据装甲板类型确定变换后的宽度
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    // 定义目标矩形的四个顶点
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    // 计算透视变换矩阵
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // 执行透视变换
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 提取数字区域ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 二值化处理
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 保存数字图像
    armor.number_img = number_image;
  }
}

/**
 * @brief 对装甲板进行数字分类
 * @param armors 装甲板列表
 * @details 使用神经网络对装甲板上的数字进行分类，并根据置信度和类型匹配进行筛选
 */
void NumberClassifier::classify(std::vector<Armor> & armors)
{
  // 对每个装甲板进行分类
  for (auto & armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // 归一化处理
    image = image / 255.0;

    // 创建blob数据
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // 设置神经网络输入
    net_.setInput(blob);
    // 前向传播获取输出
    cv::Mat outputs = net_.forward();

    // 计算softmax概率
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    // 获取最大概率对应的类别
    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    // 保存分类结果
    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    // 生成分类结果字符串
    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  // 根据置信度和类型匹配筛选装甲板
  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        // 置信度过低则剔除
        if (armor.confidence < threshold) {
          return true;
        }

        // 剔除需要忽略的类别
        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        // 检查装甲板类型与数字是否匹配
        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) {
          // 大装甲板对应的数字
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          // 小装甲板对应的数字
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace tradition_detect
}  // namespace armor_detector
