#include "GxIAPI.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <string>

namespace daheng_camera
{

class DahengCameraNode : public rclcpp::Node
{
public:
  explicit DahengCameraNode(const rclcpp::NodeOptions & options)
  : Node("daheng_camera", options), device_handle_(nullptr), use_callback_(false)
  {
    RCLCPP_INFO(this->get_logger(), "DahengCameraNode starting (adaptive mode)...");

    // ---------- 1. SDK初始化 ----------
    if (GXInitLib() != GX_STATUS_SUCCESS) {
      RCLCPP_FATAL(this->get_logger(), "Galaxy SDK init failed!");
      rclcpp::shutdown();
      return;
    }

    // ---------- 2. 枚举设备 ----------
    uint32_t device_num = 0;
    GXUpdateAllDeviceList(&device_num, 1000);
    while (device_num == 0 && rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "No camera found, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      GXUpdateAllDeviceList(&device_num, 1000);
    }
    RCLCPP_INFO(this->get_logger(), "Found %d camera(s)", device_num);

    // ---------- 3. 打开设备 ----------
    GX_OPEN_PARAM open_param;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = const_cast<char*>("1");
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    if (GXOpenDevice(&open_param, &device_handle_) != GX_STATUS_SUCCESS) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera!");
      rclcpp::shutdown();
      return;
    }

    // ---------- 4. 获取传感器尺寸 ----------
    GX_INT_VALUE int_val;
    GXGetIntValue(device_handle_, "Width", &int_val);
    img_width_ = int_val.nCurValue;
    GXGetIntValue(device_handle_, "Height", &int_val);
    img_height_ = int_val.nCurValue;
    RCLCPP_INFO(this->get_logger(), "Sensor: %ld x %ld", img_width_, img_height_);

    // ---------- 5. 像素格式自适应 ----------
    GX_ENUM_VALUE enum_val;
    GXGetEnumValue(device_handle_, "PixelFormat", &enum_val);
    int64_t fmt = enum_val.stCurValue.nCurValue;
    RCLCPP_INFO(this->get_logger(), "Default pixel format: 0x%lx", fmt);

    if (fmt == GX_PIXEL_FORMAT_BAYER_RG8) {
      encoding_ = "bgr8";
      raw_frame_size_ = img_width_ * img_height_;          // 原始数据：1 字节/像素
    } else if (fmt == GX_PIXEL_FORMAT_MONO8) {
      encoding_ = "mono8";
      raw_frame_size_ = img_width_ * img_height_;          // 原始数据：1 字节/像素
    } else {
      if (GXSetEnumValue(device_handle_, "PixelFormat", GX_PIXEL_FORMAT_BAYER_RG8) == GX_STATUS_SUCCESS) {
        encoding_ = "bgr8";
        raw_frame_size_ = img_width_ * img_height_;
      } else {
        RCLCPP_FATAL(this->get_logger(), "Unsupported format and cannot set BayerRG8.");
        rclcpp::shutdown();
        return;
      }
    }

    // ---------- 6. 通用采集设置 ----------
    GXSetEnumValueByString(device_handle_, "AcquisitionMode", "Continuous");
    GXSetEnumValueByString(device_handle_, "TriggerMode", "Off");
    GXSetEnumValueByString(device_handle_, "ExposureAuto", "Off");
    GXSetEnumValueByString(device_handle_, "GainAuto", "Off");
    GXSetFloatValue(device_handle_, "ExposureTime", 10000.0);
    GXSetFloatValue(device_handle_, "Gain", 0.0);
    GXSetFloatValue(device_handle_, "AcquisitionFrameRate", 15.0);
    GXSetEnumValue(device_handle_, "StreamBufferHandlingMode", GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST);
    GXSetIntValue(device_handle_, "StreamBufferCount", 16);

    // ---------- 7. 初始化ROS组件（先于回调启动，保证 now()/get_logger() 可用）----------
    bool use_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    declareAndLoadCalibration();
    declareParameters();
    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DahengCameraNode::parametersCallback, this, std::placeholders::_1));

    // ---------- 8. 尝试注册回调 ----------
    GX_STATUS reg_status = GXRegisterCaptureCallback(device_handle_, this, OnFrameCallback);
    if (reg_status == GX_STATUS_SUCCESS) {
      use_callback_ = true;
      RCLCPP_INFO(this->get_logger(), "Callback registered successfully.");
      // 先启动发布线程，再开流，确保帧能被及时处理
      publish_thread_ = std::thread(&DahengCameraNode::publishLoop, this);
    } else {
      RCLCPP_WARN(this->get_logger(), "Callback registration failed (code: %d), using blocking mode.", reg_status);
    }

    // ---------- 9. 开启流 ----------
    if (GXStreamOn(device_handle_) != GX_STATUS_SUCCESS) {
      RCLCPP_FATAL(this->get_logger(), "GXStreamOn failed!");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Stream started.");

    // ---------- 10. 发送采集启动命令 ----------
    GXSendCommand(device_handle_, GX_COMMAND_ACQUISITION_START);

    // ---------- 11. 阻塞模式下启动采集线程 ----------
    if (!use_callback_) {
      capture_thread_ = std::thread(&DahengCameraNode::captureLoop, this);
    }

    RCLCPP_INFO(this->get_logger(), "DahengCameraNode running, waiting for images...");
  }

  ~DahengCameraNode() override
  {
    keep_running_ = false;
    if (publish_thread_.joinable()) publish_thread_.join();
    if (capture_thread_.joinable()) capture_thread_.join();

    if (device_handle_ != nullptr) {
      GXStreamOff(device_handle_);
      GXCloseDevice(device_handle_);
    }
    GXCloseLib();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
  }

private:
  struct FrameData {
    uint32_t width, height;
    rclcpp::Time timestamp;
    std::vector<uint8_t> data;    // 原始图像数据（1 通道）
  };

  // ---------- 回调相关 ----------
  static void GX_STDC OnFrameCallback(GX_FRAME_CALLBACK_PARAM* pFrame)
  {
    if (!pFrame || !pFrame->pImgBuf) return;
    DahengCameraNode* node = static_cast<DahengCameraNode*>(pFrame->pUserParam);
    if (node) node->frameCallback(pFrame);
  }

  void frameCallback(GX_FRAME_CALLBACK_PARAM* pFrame)
  {
    if (pFrame->status != GX_FRAME_STATUS_SUCCESS) return;

    // 分辨率变化时更新原始帧大小
    if (pFrame->nWidth != img_width_ || pFrame->nHeight != img_height_) {
      img_width_ = pFrame->nWidth;
      img_height_ = pFrame->nHeight;
      raw_frame_size_ = img_width_ * img_height_;
    }

    std::lock_guard<std::mutex> lock(buf_mutex_);
    // 丢弃旧帧，防止积压
    if (frame_buffer_.size() >= 3) frame_buffer_.pop_front();

    FrameData fd;
    fd.width = pFrame->nWidth;
    fd.height = pFrame->nHeight;
    fd.timestamp = this->now();
    const uint8_t* buf = static_cast<const uint8_t*>(pFrame->pImgBuf);
    fd.data.assign(buf, buf + raw_frame_size_);
    frame_buffer_.push_back(std::move(fd));
    cv_.notify_one();
  }

  void publishLoop()
  {
    rclcpp::WallRate rate(30.0);
    while (keep_running_ && rclcpp::ok()) {
      FrameData fd;
      {
        std::unique_lock<std::mutex> lock(buf_mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(200),
                     [this]{ return !frame_buffer_.empty() || !keep_running_; });
        if (frame_buffer_.empty() || !keep_running_) continue;
        fd = std::move(frame_buffer_.front());
        frame_buffer_.pop_front();
      }
      publishFrame(fd);
    }
  }

  // ---------- 阻塞模式 ----------
  void captureLoop()
  {
    // 首帧尝试（5 次，每次 2s）
    bool first_ok = false;
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
      GX_FRAME_DATA frame;
      GX_STATUS st = GXGetImage(device_handle_, &frame, 2000);
      if (st == GX_STATUS_SUCCESS && frame.nStatus == GX_FRAME_STATUS_SUCCESS) {
        FrameData fd;
        fd.width = frame.nWidth;
        fd.height = frame.nHeight;
        fd.timestamp = this->now();
        fd.data.assign(static_cast<uint8_t*>(frame.pImgBuf),
                       static_cast<uint8_t*>(frame.pImgBuf) + frame.nWidth * frame.nHeight);
        publishFrame(fd);
        first_ok = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Blocking first frame attempt %d failed (code: %d)", i+1, st);
    }
    if (!first_ok) {
      RCLCPP_FATAL(this->get_logger(), "Cannot acquire first frame in blocking mode, exiting.");
      rclcpp::shutdown();
      return;
    }

    while (keep_running_ && rclcpp::ok()) {
      GX_FRAME_DATA frame;
      GX_STATUS st = GXGetImage(device_handle_, &frame, 1000);
      if (st == GX_STATUS_SUCCESS && frame.nStatus == GX_FRAME_STATUS_SUCCESS) {
        FrameData fd;
        fd.width = frame.nWidth;
        fd.height = frame.nHeight;
        fd.timestamp = this->now();
        fd.data.assign(static_cast<uint8_t*>(frame.pImgBuf),
                       static_cast<uint8_t*>(frame.pImgBuf) + frame.nWidth * frame.nHeight);
        publishFrame(fd);
      } else {
        RCLCPP_WARN(this->get_logger(), "GetImage failed: %d", st);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  void publishFrame(const FrameData& fd)
  {
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = fd.timestamp;
    img_msg.header.frame_id = "camera_optical_frame";
    img_msg.width = fd.width;
    img_msg.height = fd.height;
    img_msg.encoding = encoding_;
    img_msg.step = img_msg.width * (encoding_ == "bgr8" ? 3 : 1);
    img_msg.data.resize(img_msg.step * img_msg.height);

    if (encoding_ == "bgr8") {
      cv::Mat bayer(fd.height, fd.width, CV_8UC1, const_cast<uint8_t*>(fd.data.data()));
      cv::Mat rgb(fd.height, fd.width, CV_8UC3, img_msg.data.data());
      cv::cvtColor(bayer, rgb, cv::COLOR_BayerRG2BGR);
    } else {
      std::memcpy(img_msg.data.data(), fd.data.data(), fd.data.size());
    }

    camera_info_msg_.header = img_msg.header;
    camera_pub_.publish(img_msg, camera_info_msg_);
  }

  // ---------- 辅助函数 ----------
  void declareAndLoadCalibration()
  {
    camera_name_ = this->declare_parameter("camera_name", "daheng_camera");
    auto url = this->declare_parameter("camera_info_url", "");
    if (url.empty()) {
      RCLCPP_INFO(this->get_logger(), "No calibration URL, using identity.");
      return;
    }
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    if (!camera_info_manager_->validateURL(url)) {
      RCLCPP_WARN(this->get_logger(), "Invalid calibration URL.");
      return;
    }
    if (camera_info_manager_->loadCameraInfo(url)) {
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
      RCLCPP_INFO(this->get_logger(), "Calibration loaded from: %s", url.c_str());
    }
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    GX_FLOAT_VALUE fv;

    // ---------- 曝光时间 ----------
    GXGetFloatValue(device_handle_, "ExposureTime", &fv);
    desc.floating_point_range.resize(1);
    desc.floating_point_range[0].from_value = fv.dMin;
    desc.floating_point_range[0].to_value = fv.dMax;
    desc.floating_point_range[0].step = 0.0;
    double expo = this->declare_parameter("exposure_time", fv.dCurValue, desc);
    GXSetFloatValue(device_handle_, "ExposureTime", expo);

    // ---------- 增益 ----------
    GXGetFloatValue(device_handle_, "Gain", &fv);
    desc.floating_point_range[0].from_value = fv.dMin;
    desc.floating_point_range[0].to_value = fv.dMax;
    double gain = this->declare_parameter("gain", fv.dCurValue, desc);
    GXSetFloatValue(device_handle_, "Gain", gain);

    GXSetEnumValueByString(device_handle_, "ExposureAuto", "Off");
    GXSetEnumValueByString(device_handle_, "GainAuto", "Off");

    // ==================== 白平衡控制（直接尝试，兼容所有型号）====================
    // 1. 白平衡自动模式
    GX_STATUS wb_status = GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", "Off");
    if (wb_status == GX_STATUS_SUCCESS) {
        std::string wb_mode = "Off";
        GX_ENUM_VALUE wb_enum;
        if (GXGetEnumValue(device_handle_, "BalanceWhiteAuto", &wb_enum) == GX_STATUS_SUCCESS) {
            if (wb_enum.stCurValue.nCurValue == 1) wb_mode = "Continuous";
            else if (wb_enum.stCurValue.nCurValue == 2) wb_mode = "Once";
        }
        rcl_interfaces::msg::ParameterDescriptor wb_desc;
        wb_desc.description = "White balance auto mode: Off, Once, Continuous";
        this->declare_parameter("whitebalance_auto", wb_mode, wb_desc);
        GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", wb_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "White balance auto initialized: %s", wb_mode.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "BalanceWhiteAuto not supported, skipping WB auto parameter.");
    }

        // 2. 手动白平衡比率（红/蓝）
    GX_STATUS ratio_status = GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Red");
    if (ratio_status == GX_STATUS_SUCCESS) 
    {
      GX_FLOAT_VALUE bf;
      if (GXGetFloatValue(device_handle_, "BalanceRatio", &bf) == GX_STATUS_SUCCESS) 
      {
        // 声明红色比率参数
        rcl_interfaces::msg::ParameterDescriptor red_desc;
        red_desc.description = "White balance red ratio";
        red_desc.floating_point_range.resize(1);
        red_desc.floating_point_range[0].from_value = bf.dMin;
        red_desc.floating_point_range[0].to_value = bf.dMax;
        red_desc.floating_point_range[0].step = 0.0;
        double red_ratio = this->declare_parameter("wb_red_ratio", bf.dCurValue, red_desc);
        // 注意：声明后立即写入，但此时可能被 launch 覆盖，所以用 get_parameter 修正（下面会重写）
        GXSetFloatValue(device_handle_, "BalanceRatio", red_ratio);

        // 声明蓝色比率参数
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Blue");
        GXGetFloatValue(device_handle_, "BalanceRatio", &bf);
        rcl_interfaces::msg::ParameterDescriptor blue_desc;
        blue_desc.description = "White balance blue ratio";
        blue_desc.floating_point_range.resize(1);
        blue_desc.floating_point_range[0].from_value = bf.dMin;
        blue_desc.floating_point_range[0].to_value = bf.dMax;
        blue_desc.floating_point_range[0].step = 0.0;
        double blue_ratio = this->declare_parameter("wb_blue_ratio", bf.dCurValue, blue_desc);
        GXSetFloatValue(device_handle_, "BalanceRatio", blue_ratio);

        // 声明绿色比率参数（新增）
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Green");
        GXGetFloatValue(device_handle_, "BalanceRatio", &bf);
        rcl_interfaces::msg::ParameterDescriptor green_desc;
        green_desc.description = "White balance green ratio";
        green_desc.floating_point_range.resize(1);
        green_desc.floating_point_range[0].from_value = bf.dMin;
        green_desc.floating_point_range[0].to_value = bf.dMax;
        green_desc.floating_point_range[0].step = 0.0;
        double green_ratio = this->declare_parameter("wb_green_ratio", bf.dCurValue, green_desc);
        GXSetFloatValue(device_handle_, "BalanceRatio", green_ratio);

        // 上面声明时可能被 launch 覆盖，所以统一再读取一次并强制应用
        double red_final = this->get_parameter("wb_red_ratio").as_double();
        double blue_final = this->get_parameter("wb_blue_ratio").as_double();
        double green_final = this->get_parameter("wb_green_ratio").as_double();
        
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Red");
        GXSetFloatValue(device_handle_, "BalanceRatio", red_final);
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Blue");
        GXSetFloatValue(device_handle_, "BalanceRatio", blue_final);
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Green");
        GXSetFloatValue(device_handle_, "BalanceRatio", green_final);

        RCLCPP_INFO(this->get_logger(), "Manual WB ratios: Red=%.3f, Blue=%.3f, Green=%.3f",
                    red_final, blue_final, green_final);
      }
    } 
    else 
    {
      RCLCPP_INFO(this->get_logger(), "BalanceRatio not supported, skipping manual WB parameters.");
    }
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto& p : params) {
      if (p.get_name() == "exposure_time") {
        if (GXSetFloatValue(device_handle_, "ExposureTime", p.as_double()) != GX_STATUS_SUCCESS) {
          res.successful = false;
          res.reason = "Failed to set exposure";
        }
      } else if (p.get_name() == "gain") {
        if (GXSetFloatValue(device_handle_, "Gain", p.as_double()) != GX_STATUS_SUCCESS) {
          res.successful = false;
          res.reason = "Failed to set gain";
        }
      }
      // ---------- 白平衡参数处理（新增） ----------
      else if (p.get_name() == "whitebalance_auto") {
        if (GXSetEnumValueByString(device_handle_, "BalanceWhiteAuto", p.as_string().c_str()) != GX_STATUS_SUCCESS) {
          res.successful = false;
          res.reason = "Failed to set BalanceWhiteAuto";
        }
      } else if (p.get_name() == "wb_red_ratio") {
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Red");
        if (GXSetFloatValue(device_handle_, "BalanceRatio", p.as_double()) != GX_STATUS_SUCCESS) {
          res.successful = false;
          res.reason = "Failed to set red balance ratio";
        }
      } else if (p.get_name() == "wb_blue_ratio") {
        GXSetEnumValueByString(device_handle_, "BalanceRatioSelector", "Blue");
        if (GXSetFloatValue(device_handle_, "BalanceRatio", p.as_double()) != GX_STATUS_SUCCESS) {
          res.successful = false;
          res.reason = "Failed to set blue balance ratio";
        }
      }
      else {
        res.successful = false;
        res.reason = "Unknown param: " + p.get_name();
      }

      if (!res.successful) {
        // 一旦有错误，立即返回（可累积，这里简单处理）
        return res;
      }
    }
    return res;
  }

  // ---------- 成员变量 ----------
  GX_DEV_HANDLE device_handle_;
  int64_t img_width_ = 0, img_height_ = 0;
  size_t raw_frame_size_ = 0;          // 原始图像字节数（1 通道）
  std::string encoding_;

  bool use_callback_;
  std::deque<FrameData> frame_buffer_;
  std::mutex buf_mutex_;
  std::condition_variable cv_;
  std::thread publish_thread_;
  std::thread capture_thread_;
  std::atomic<bool> keep_running_{true};

  image_transport::CameraPublisher camera_pub_;
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

} // namespace daheng_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(daheng_camera::DahengCameraNode)