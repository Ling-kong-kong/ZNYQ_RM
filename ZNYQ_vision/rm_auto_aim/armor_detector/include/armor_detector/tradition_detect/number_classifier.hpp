// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/tradition_detect/armor.hpp"

namespace armor_detector
{
namespace tradition_detect
{
class NumberClassifier
{
public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void classify(std::vector<Armor> & armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace tradition_detect
}  // namespace armor_detector

// 为了保持向后兼容性，在rm_auto_aim命名空间中创建类型别名
namespace rm_auto_aim
{
using NumberClassifier = armor_detector::tradition_detect::NumberClassifier;
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
