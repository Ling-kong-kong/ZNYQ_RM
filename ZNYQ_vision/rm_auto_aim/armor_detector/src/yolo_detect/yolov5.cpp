/**
 * @file yolov5.cpp
 * @brief YOLOv5装甲板检测器实现
 * @details 基于YOLOv5神经网络模型实现装甲板检测功能，包括：
 *          - 模型初始化与配置加载
 *          - 图像预处理与推理
 *          - 后处理与结果解析
 *          - 装甲板验证与过滤
 */

#include "armor_detector/yolo_detect/yolov5.hpp"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

namespace armor_detector
{
namespace yolo_detect
{
  /**
   * @brief YOLOv5构造函数
   * @param config_path 配置文件路径
   * @param debug 是否开启调试模式
   * @param use_roi 是否使用ROI区域
   * @param roi ROI区域参数
   * @details 初始化YOLOv5检测器，包括：
   *          1. 从YAML配置文件加载参数
   *          2. 设置ROI区域和设备配置
   *          3. 配置OpenVINO推理引擎
   *          4. 设置输入预处理流程
   */
  YOLOV5::YOLOV5(const std::string &config_path, bool debug, bool use_roi, const cv::Rect &roi)
      : debug_(debug), use_roi_(use_roi), roi_(roi)
  {
    // 从配置文件加载参数
    auto yaml = YAML::LoadFile(config_path);

    std::cout << "Loading config from: " << config_path << std::endl;

    // 安全地加载参数，添加错误处理
    try
    {
      model_path_ = yaml["yolov5_model_path"].as<std::string>();
      device_ = yaml["device"].as<std::string>();
      // binary_threshold_ = yaml["threshold"].as<double>();
      min_confidence_ = yaml["min_confidence"].as<double>();
      // use_traditional_ = yaml["use_traditional"].as<bool>();

      std::cout << "Config loaded successfully:" << std::endl;
      std::cout << "  model_path: " << model_path_ << std::endl;
      std::cout << "  device: " << device_ << std::endl;
      // std::cout << "  threshold: " << binary_threshold_ << std::endl;
      std::cout << "  min_confidence: " << min_confidence_ << std::endl;
      // std::cout << "  use_traditional: " << use_traditional_ << std::endl;
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error loading config: " << e.what() << std::endl;
      throw;
    }

    // 处理模型文件的相对路径
    if (model_path_[0] != '/')
    {
      // 获取包的共享目录路径
      std::string package_share_dir = ament_index_cpp::get_package_share_directory("armor_detector");
      std::string temp_path = package_share_dir + "/models/yolo_detect/assets/" + std::filesystem::path(model_path_).filename().string();

      if (std::filesystem::exists(temp_path))
      {
        model_path_ = temp_path;
      }
      else
      {
        // 如果找不到，使用原始相对路径
        std::filesystem::path config_file_path(config_path);
        std::string config_dir = config_file_path.parent_path().string();
        model_path_ = config_dir + "/" + model_path_;
      }
    }

    // 设置ROI偏移量
    offset_ = cv::Point2f(roi_.x, roi_.y);

    // 调试输出
    std::cout << "Loading model from: " << model_path_ << std::endl;
    if (!std::filesystem::exists(model_path_))
    {
      std::cerr << "Model file does not exist: " << model_path_ << std::endl;
      throw std::runtime_error("Model file not found: " + model_path_);
    }

    // 加载和配置神经网络模型
    auto model = core_.read_model(model_path_);
    ov::preprocess::PrePostProcessor ppp(model);
    auto &input = ppp.input();

    // 配置输入张量：数据类型、形状、布局和颜色格式
    input.tensor()
        .set_element_type(ov::element::u8)                   // uint8类型
        .set_shape({1, 640, 640, 3})                         // NHWC格式，640x640分辨率
        .set_layout("NHWC")                                  // 批次-高度-宽度-通道
        .set_color_format(ov::preprocess::ColorFormat::BGR); // BGR颜色格式

    // 设置模型输入布局为NCHW（批次-通道-高度-宽度）
    input.model().set_layout("NCHW");

    // 配置预处理流程：数据类型转换、颜色空间转换、归一化
    input.preprocess()
        .convert_element_type(ov::element::f32)          // 转换为float32
        .convert_color(ov::preprocess::ColorFormat::RGB) // BGR转RGB
        .scale(255.0);                                   // 归一化到[0,1]

    // 构建预处理后的模型并编译
    model = ppp.build();
    compiled_model_ = core_.compile_model(
        model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  }

  /**
   * @brief 装甲板检测主函数
   * @param raw_img 原始输入图像
   * @param frame_count 帧计数器
   * @return 检测到的装甲板列表
   * @details 完整的检测流程：
   *          1. 图像有效性检查
   *          2. ROI区域裁剪（如果启用）
   *          3. 图像预处理（缩放、填充）
   *          4. 神经网络推理
   *          5. 结果后处理
   */
  std::list<Armor> YOLOV5::detect(const cv::Mat &raw_img, int frame_count)
  {
    // 检查图像是否为空
    if (raw_img.empty())
    {
      std::cerr << "Warning: Empty image, camera drop!" << std::endl;
      return std::list<Armor>();
    }

    // ROI区域处理
    cv::Mat bgr_img;
    if (use_roi_)
    {
      // 动态调整ROI尺寸（-1表示不裁切该维度）
      if (roi_.width == -1)
      {
        roi_.width = raw_img.cols;
      }
      if (roi_.height == -1)
      {
        roi_.height = raw_img.rows;
      }
      bgr_img = raw_img(roi_); // 裁剪ROI区域
    }
    else
    {
      bgr_img = raw_img;
    }

    // 计算缩放比例，保持宽高比
    auto x_scale = static_cast<double>(640) / bgr_img.rows;
    auto y_scale = static_cast<double>(640) / bgr_img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    // 图像预处理：创建640x640的输入张量，居中放置缩放后的图像
    auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0)); // 黑色背景
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, input(roi), {w, h}); // 缩放并放置到中心
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

    // 神经网络推理
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // 后处理：获取输出张量并转换为OpenCV格式
    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    return parse(scale, output, raw_img, frame_count);
  }

  /**
   * @brief 解析神经网络输出结果
   * @param scale 缩放比例
   * @param output 神经网络输出张量
   * @param bgr_img 原始BGR图像
   * @param frame_count 帧计数器
   * @return 解析后的装甲板列表
   * @details 后处理流程：
   *          1. 解析每个检测框的坐标、类别和置信度
   *          2. 提取装甲板的4个角点坐标
   *          3. 应用非极大值抑制（NMS）去除重复检测
   *          4. 验证检测结果的有效性
   *          5. 可选的传统方法二次矫正
   */
  std::list<Armor> YOLOV5::parse(
      double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count)
  {
    // 存储检测结果的数据结构
    std::vector<int> color_ids, num_ids;                     // 颜色ID和数字ID
    std::vector<float> confidences;                          // 置信度
    std::vector<cv::Rect> boxes;                             // 边界框
    std::vector<std::vector<cv::Point2f>> armors_key_points; // 装甲板角点

    // 遍历输出张量的每一行（每个检测候选）
    for (int r = 0; r < output.rows; r++)
    {
      // 获取目标置信度并应用sigmoid激活函数
      double score = output.at<float>(r, 8);
      score = sigmoid(score);

      // 过滤低置信度检测
      if (score < score_threshold_)
        continue;

      std::vector<cv::Point2f> armor_key_points;

      // 解析颜色和类别独热编码向量
      cv::Mat color_scores = output.row(r).colRange(9, 13);    // 颜色分数 [9-12]
      cv::Mat classes_scores = output.row(r).colRange(13, 22); // 数字分数 [13-21]
      cv::Point class_id, color_id;
      int _class_id, _color_id;
      double score_color, score_num;
      cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
      cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
      _class_id = class_id.x;
      _color_id = color_id.x;

      // 提取装甲板4个角点坐标并还原到原图尺寸
      // 注意：YOLOv5输出的角点顺序可能需要调整
      armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
      armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
      armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
      armor_key_points.push_back(
          cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

      // 计算包围盒
      float min_x = armor_key_points[0].x;
      float max_x = armor_key_points[0].x;
      float min_y = armor_key_points[0].y;
      float max_y = armor_key_points[0].y;

      for (size_t i = 1; i < armor_key_points.size(); i++)
      {
        if (armor_key_points[i].x < min_x)
          min_x = armor_key_points[i].x;
        if (armor_key_points[i].x > max_x)
          max_x = armor_key_points[i].x;
        if (armor_key_points[i].y < min_y)
          min_y = armor_key_points[i].y;
        if (armor_key_points[i].y > max_y)
          max_y = armor_key_points[i].y;
      }

      cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

      // 存储检测结果
      color_ids.emplace_back(_color_id);
      num_ids.emplace_back(_class_id);
      boxes.emplace_back(rect);
      confidences.emplace_back(score);
      armors_key_points.emplace_back(armor_key_points);
    }

    // 应用非极大值抑制去除重叠检测
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    // 构建最终的装甲板列表
    std::list<Armor> armors;
    for (const auto &i : indices)
    {
      if (use_roi_)
      {
        // 考虑ROI偏移
        armors.emplace_back(
            color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
      }
      else
      {
        armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
      }
    }

    // 验证和后处理每个装甲板
    tmp_img_ = bgr_img;
    for (auto it = armors.begin(); it != armors.end();)
    {
      // 验证装甲板名称有效性
      if (!check_name(*it))
      {
        it = armors.erase(it);
        continue;
      }

      // 验证装甲板类型一致性
      if (!check_type(*it))
      {
        it = armors.erase(it);
        continue;
      }

      // 传统方法矫正功能暂时禁用
      // if (use_traditional_) detector_.detect(*it, bgr_img);

      // 计算归一化中心坐标
      it->center_norm = get_center_norm(bgr_img, it->center);
      ++it;
    }

    // 调试模式下绘制检测结果
    // if (debug_)
    // {
    //   draw_detections(bgr_img, armors, frame_count);
    // }

    return armors;
  }

  /**
   * @brief 验证装甲板名称有效性
   * @param armor 待验证的装甲板
   * @return 是否通过验证
   * @details 检查装甲板是否为有效类型且置信度满足要求
   */
  bool YOLOV5::check_name(const Armor &armor) const
  {
    auto name_ok = armor.name != ArmorName::not_armor;       // 不是无效装甲板
    auto confidence_ok = armor.confidence > min_confidence_; // 置信度足够高

    // 保存不确定的图案，用于神经网络的迭代优化（当前注释掉）
    // if (name_ok && !confidence_ok) save(armor);

    return name_ok && confidence_ok;
  }

  /**
   * @brief 验证装甲板类型一致性
   * @param armor 待验证的装甲板
   * @return 是否通过验证
   * @details 检查装甲板类型与数字标识的逻辑一致性：
   *          - 小装甲板：不能是数字1或基地
   *          - 大装甲板：不能是数字2、哨兵或前哨站
   */
  bool YOLOV5::check_type(const Armor &armor) const
  {
    auto name_ok = (armor.type == ArmorType::small)
                       ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                       : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                          armor.name != ArmorName::outpost);

    // 保存异常的图案，用于神经网络的迭代优化（当前注释掉）
    // if (!name_ok) save(armor);

    return name_ok;
  }

  /**
   * @brief 计算归一化中心坐标
   * @param bgr_img 输入图像
   * @param center 装甲板中心点
   * @return 归一化坐标 (x/w, y/h)
   */
  cv::Point2f YOLOV5::get_center_norm(const cv::Mat &bgr_img, const cv::Point2f &center) const
  {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
  }

  /**
   * @brief 绘制检测结果
   * @param img 原始图像
   * @param armors 检测到的装甲板列表
   * @param frame_count 帧计数器
   * @details 在图像上绘制：
   *          - 帧编号
   *          - 装甲板边界框和角点
   *          - 置信度、颜色、数字和类型信息
   *          - ROI区域（如果启用）
   */
  // void YOLOV5::draw_detections(
  //     const cv::Mat &img, const std::list<Armor> &armors, int frame_count) const
  // {
  //   auto detection = img.clone();
  //   // 绘制帧编号
  //   cv::putText(detection, std::string("[") + std::to_string(frame_count) + std::string("]"),
  //               cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

  //   // 绘制每个装甲板的检测信息
  //   for (const auto &armor : armors)
  //   {
  //     std::string info = std::to_string(armor.confidence).substr(0, 4) + " " +
  //                        COLORS[armor.color] + " " + ARMOR_NAMES[armor.name] + " " +
  //                        ARMOR_TYPES[armor.type];

  //     // 绘制角点
  //     for (size_t i = 0; i < armor.points.size(); ++i)
  //     {
  //       cv::circle(detection, armor.points[i], 3, cv::Scalar(0, 255, 0), -1);
  //     }

  //     // 绘制信息文本
  //     cv::putText(detection, info, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.6,
  //                 cv::Scalar(0, 255, 0), 2);
  //   }

  //   // 绘制ROI区域
  //   if (use_roi_)
  //   {
  //     cv::Scalar green(0, 255, 0);
  //     cv::rectangle(detection, roi_, green, 2);
  //   }

  //   // 缩放显示以提高性能
  //   cv::resize(detection, detection, {}, 0.5, 0.5);
  //   cv::imshow("detection", detection);
  // }

  

  /**
   * @brief Sigmoid激活函数
   * @param x 输入值
   * @return sigmoid(x) = 1/(1+exp(-x))
   * @details 数值稳定的实现，避免大数时的溢出
   */
  double YOLOV5::sigmoid(double x)
  {
    if (x > 0)
      return 1.0 / (1.0 + exp(-x));
    else
      return exp(x) / (1.0 + exp(x));
  }

  /**
   * @brief 更新ROI设置
   * @param roi 新的ROI区域
   * @param use_roi 是否使用ROI
   */
  void YOLOV5::update_roi(const cv::Rect &roi, bool use_roi)
  {
    roi_ = roi;
    use_roi_ = use_roi;
    offset_ = cv::Point2f(roi_.x, roi_.y);

    if (debug_)
    {
      std::cout << "ROI updated to: (" << roi_.x << ", " << roi_.y << ", "
                << roi_.width << ", " << roi_.height << "), use_roi: "
                << (use_roi_ ? "true" : "false") << std::endl;
    }
  }

  /**
   * @brief 后处理接口函数
   * @param scale 缩放比例
   * @param output 神经网络输出
   * @param bgr_img 输入图像
   * @param frame_count 帧计数器
   * @return 处理后的装甲板列表
   * @details 调用parse函数进行实际的后处理
   */
  std::list<Armor> YOLOV5::postprocess(
      double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count)
  {
    return parse(scale, output, bgr_img, frame_count);
  }

} // namespace yolo_detect
} // namespace armor_detector