# yolo_detect 包 - 类和函数功能总结

本文档根据提供的源代码总结了 `yolo_detect` 包中类和函数的功能。

## 命名空间 `yolo_detect`

### 枚举

-   **`Color`**: 定义可能的装甲板颜色：`red`（红色）、`blue`（蓝色）、`extinguish`（熄灭）、`purple`（紫色）。
-   **`ArmorType`**: 定义装甲板类型：`big`（大装甲板）、`small`（小装甲板）。
-   **`ArmorName`**: 定义装甲板名称/编号：`one`（1号）、`two`（2号）、`three`（3号）、`four`（4号）、`five`（5号）、`sentry`（哨兵）、`outpost`（前哨站）、`base`（基地）、`not_armor`（非装甲板）。
-   **`ArmorPriority`**: 定义装甲板优先级：`first`（第一）、`second`（第二）、`third`（第三）、`forth`（第四）、`fifth`（第五）。

### 结构体 `Lightbar`

**目的**: 存储检测到的灯条的几何属性和识别细节。

-   **`Lightbar(const cv::RotatedRect &rotated_rect, std::size_t id)`**:
    -   **功能**: 构造函数，使用 `cv::RotatedRect`（表示检测到的灯条形状）和一个唯一 ID 初始化 `Lightbar` 对象。它计算各种几何属性，如中心点、顶部/底部点、长度、宽度和角度。
-   **`Lightbar()`**:
    -   **功能**: 默认构造函数。

### 结构体 `Armor`

**目的**: 存储装甲板的综合属性，包括其几何信息、识别结果（颜色、类型、名称、置信度）和空间位置估计。

-   **`Armor(int color_id, int num_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints)`**:
    -   **功能**: 专门设计用于解释 YOLOv5 类检测器原始输出的构造函数。它接收颜色 ID、数字 ID、置信度分数、边界框以及装甲板的四个角点。然后计算装甲板的中心、宽度、长度、滚转角、矩形误差和长宽比。它还将整数 `color_id` 和 `num_id` 转换为相应的 `Color`、`ArmorName` 和 `ArmorType` 枚举。
-   **`Armor(int color_id, int num_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)`**:
    -   **功能**: 一个重载构造函数，通过引入 `offset` 扩展了上一个构造函数的功能。当在感兴趣区域 (ROI) 内进行检测时使用此函数，并且需要通过添加 `offset` 将坐标（关键点和边界框）从 ROI 的局部坐标系转换回完整图像的全局坐标系。

### 类 `YoloDetect` (ROS 2 节点)

**目的**: 此类定义了一个 ROS 2 节点（`rclcpp::Node`），负责协调装甲板检测过程。它处理 ROS 2 通信（订阅、发布）并集成 YOLOv5 检测器。

-   **`YoloDetect()`**:
    -   **功能**: `YoloDetect` 节点的主要构造函数。它初始化 ROS 节点，声明并检索配置参数（例如，模型路径、调试模式、图像话题、ROI 设置），初始化内部 `YOLOV5` 检测器，设置图像订阅者（`/image_raw`），以及可视化结果的发布者（`/result_img`）。它还注册参数回调以进行动态配置。
-   **`void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)`**:
    -   **功能**: 这是当在订阅的图像话题上接收到新的 `sensor_msgs/Image` 消息时调用的回调函数。它将 ROS 图像转换为 OpenCV `cv::Mat`，调用 `YOLOV5` 检测器的 `detect` 方法，计算并更新检测帧率 (FPS)，将 `yolo_detect::Armor` 对象转换为 `rm_auto_aim::Armor`（用于兼容性），如果启用了调试模式，则绘制并发布可视化结果。
-   **`cv::Mat draw_detections(const cv::Mat &img, const std::list<yolo_detect::Armor> &armors, double fps, long detection_time)`**:
    -   **功能**: 通过将检测结果（边界框、关键点、置信度、颜色、名称、类型）和调试信息（FPS、检测时间、ROI 区域）叠加到原始输入图像上，生成一个新的 `cv::Mat` 图像。此结果图像适用于可视化。
-   **`void publish_result_image(const cv::Mat &img, const std_msgs::msg::Header &header)`**:
    -   **功能**: 将 OpenCV `cv::Mat`（通常是 `draw_detections` 生成的可视化图像）转换回 `sensor_msgs/Image` 消息，并将其发布到 `/result_img` 话题。
-   **`bool validate_roi_bounds(int x, int y, int width, int height)`**:
    -   **功能**: 一个实用函数，用于检查由其左上角（`x`, `y`）和尺寸（`width`, `height`）定义的给定感兴趣区域 (ROI) 是否完全位于图像的已知边界（`image_width_`、`image_height_`）内。它确保在应用 ROI 参数之前这些参数是有效的。
-   **`bool initYolo()`**:
    -   **功能**: 处理 YOLO 检测器的初始设置和参数声明。它从 ROS 参数服务器和 YAML 配置文件（`sentry.yaml`）检索参数，处理配置文件的相对路径，验证 ROI 参数，最后构建并初始化 `YOLOV5` 检测器对象。
-   **`rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)`**:
    -   **功能**: 当 ROS 2 参数被设置时触发的通用回调函数。它遍历接收到的参数，更新相应的成员变量（`use_roi_`、`roi_.x`、`roi_.y`、`roi_.width`、`roi_.height`），然后调用 `detector_->update_roi()` 将这些更改传播到底层 YOLOv5 检测器。它记录更新。
-   **`rcl_interfaces::msg::SetParametersResult roi_parameter_callback(const std::vector<rclcpp::Parameter> &parameters)`**:
    -   **功能**: 一个更具体的参数回调，主要侧重于更新 ROI 相关参数。它在更新 `roi_` 和 `use_roi_` 之前对新的 ROI 尺寸进行图像边界验证。如果验证通过，它会更新 `YOLOV5` 检测器的 ROI 设置。此回调确保动态 ROI 调整是安全有效的。
-   **`rm_auto_aim::Armor convert_to_rm_armor(const yolo_detect::Armor& source_armor)`**:
    -   **功能**: 将 `yolo_detect::Armor` 结构（包内部）转换为 `rm_auto_aim::Armor` 结构。此转换可能对于与独立的 RoboMaster 自动瞄准系统接口是必需的，以确保数据兼容性。它映射检测细节，如中心、类型、置信度、数字和分类结果。它还根据装甲板的角点估算灯条信息（中心、长度、角度）。

### 类 `YOLOV5` (YOLOv5 检测器)

**目的**: 此类封装了核心 YOLOv5 对象检测逻辑，包括模型加载、推理和使用 OpenVINO 工具包进行后处理。

-   **`YOLOV5(const std::string & config_path, bool debug, bool use_roi, const cv::Rect & roi)`**:
    -   **功能**: 初始化 YOLOv5 检测器的构造函数。它从 YAML 文件加载配置参数（模型路径、设备、阈值），解析模型文件路径，通过读取和预处理 YOLOv5 模型来设置 OpenVINO 推理引擎，并为指定的硬件设备编译模型。它还初始化调试、ROI 使用和 ROI 参数。
-   **`std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count)`**:
    -   **功能**: 对输入图像执行对象检测的主要入口点。它首先检查空图像，然后选择性地将图像裁剪到定义的 ROI。它将图像大小调整并填充到模型预期的输入尺寸（640x640）。图像预处理后，它使用 OpenVINO 执行神经网络推理，然后调用 `parse` 方法处理原始模型输出。
-   **`std::list<Armor> postprocess(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)`**:
    -   **功能**: 一个接口函数，将实际的后处理委托给 `parse` 方法。它可能用于 API 一致性或未来的扩展。
-   **`void update_roi(const cv::Rect &roi, bool use_roi)`**:
    -   **功能**: 动态更新检测器内部的 `roi_`（感兴趣区域矩形）和 `use_roi_` 标志。它还根据新的 ROI 重新计算 `offset_`，并在调试模式下记录更新。
-   **`bool check_name(const Armor & armor) const`**:
    -   **功能**: 对检测到的 `Armor` 对象执行验证检查。如果装甲板的 `name` 不是 `not_armor` 并且其 `confidence` 超过预定义的 `min_confidence_` 阈值，则返回 `true`。
-   **`bool check_type(const Armor & armor) const`**:
    -   **功能**: 验证装甲板的检测到的 `type` 和 `name` 之间的逻辑一致性。例如，它检查小装甲板是否被错误地分类为“一号”或“基地”，或者大装甲板是否被分类为“二号”、“哨兵”或“前哨站”。这有助于过滤掉逻辑上不一致的检测。
-   **`cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const`**:
    -   **功能**: 计算给定 `center` 点相对于 `bgr_img` 尺寸的归一化坐标（`x/宽度`，`y/高度`）。这对于独立于图像分辨率表示位置很有用。
-   **`std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)`**:
    -   **功能**: 这是核心后处理方法。它接收神经网络的原始输出张量，遍历每个检测到的候选，对置信度分数应用 sigmoid 激活，并过滤掉低置信度检测。它提取每个装甲板的边界框坐标、颜色 ID、数字 ID 和四个关键点。然后应用非极大值抑制 (NMS) 以消除冗余边界框。最后，它构建 `yolo_detect::Armor` 对象列表，如果启用 ROI，则应用 ROI 偏移，并在返回最终检测到的装甲板列表之前执行 `check_name` 和 `check_type` 验证。
-   **`void save(const Armor & armor) const`**:
    -   **功能**: （当前已注释掉）此函数旨在将检测到的装甲板图像保存到指定目录，可能用于数据集收集或进一步分析。
-   **`double sigmoid(double x)`**:
    -   **功能**: 实现了一个数值稳定的 sigmoid 激活函数 `1 / (1 + exp(-x))`，用于将神经网络的原始分数转换为概率或置信度值。