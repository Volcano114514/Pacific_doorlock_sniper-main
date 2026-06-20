#include <rclcpp/rclcpp.hpp>
#include <doorlock_sniper/msg/video_packet.hpp>
#include <serial_driver/serial_port.hpp>   // 不再需要 serial_driver.hpp
#include <memory>                          // for std::make_shared

using namespace drivers::serial_driver;
using namespace drivers::common;          // IoContext

// 固定协议常量
const uint8_t FRAME_HEADER[2] = {0xAA, 0x55};
const uint8_t FRAME_TAIL = 0x0D;
const size_t DATA_SIZE = 300;
const size_t TOTAL_FRAME_SIZE = 320; // 2+8+8+300+1+1

class VideoUartSender : public rclcpp::Node
{
public:
  VideoUartSender()
  : Node("video_uart_sender"),
    io_ctx_()                              // IoContext 作为成员
  {
    // 声明并读取 ROS 参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 921600);

    std::string serial_port = this->get_parameter("serial_port").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();

    // 串口配置
    SerialPortConfig config(
      baud_rate,
      FlowControl::NONE,
      Parity::NONE,
      StopBits::ONE
    );

    try {
      // 直接创建 SerialPort 对象，同时传入设备名和配置
      serial_port_ = std::make_shared<SerialPort>(io_ctx_, serial_port, config);
      serial_port_->open();  // 无参打开
      RCLCPP_INFO(this->get_logger(), "串口 %s 打开成功，波特率: %d",
                  serial_port.c_str(), baud_rate);
    } catch (const std::runtime_error& e) {
      RCLCPP_FATAL(this->get_logger(), "串口打开失败: %s\n"
                   "请检查：\n"
                   "1. 串口设备名是否正确（ls /dev/ttyACM* /dev/ttyUSB*）\n"
                   "2. 当前用户是否有串口权限（sudo usermod -aG dialout $USER）\n"
                   "3. 串口是否被其他程序占用",
                   e.what());
      rclcpp::shutdown();
      return;
    }

    // 订阅视频流
    video_sub_ = this->create_subscription<doorlock_sniper::msg::VideoPacket>(
      "video_stream",
      rclcpp::QoS(rclcpp::KeepLast(3000)).reliable(),
      std::bind(&VideoUartSender::video_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "视频UART发送节点已启动，帧长: %zu字节", TOTAL_FRAME_SIZE);
  }

  ~VideoUartSender() override
  {
    if (serial_port_ && serial_port_->is_open()) {
      serial_port_->close();
    }
  }

private:
  void video_callback(const doorlock_sniper::msg::VideoPacket::SharedPtr msg)
  {
    if (!serial_port_->is_open()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "串口未打开，跳过本次发送");
      return;
    }

    try {
      std::vector<uint8_t> frame(TOTAL_FRAME_SIZE);
      size_t offset = 0;

      // 1. 帧头
      memcpy(frame.data() + offset, FRAME_HEADER, 2);
      offset += 2;

      // 2. sequence_id (8字节，小端序)
      memcpy(frame.data() + offset, &msg->sequence_id, 8);
      offset += 8;

      // 3. timestamp_ns (8字节，小端序)
      memcpy(frame.data() + offset, &msg->timestamp_ns, 8);
      offset += 8;

      // 4. 视频数据 (300字节)
      memcpy(frame.data() + offset, msg->data.data(), DATA_SIZE);
      offset += DATA_SIZE;

      // 5. 校验和
      uint8_t checksum = 0;
      for (size_t i = 0; i < offset; ++i) {
        checksum ^= frame[i];
      }
      frame[offset++] = checksum;

      // 6. 帧尾
      frame[offset++] = FRAME_TAIL;

      // 7. 发送
      serial_port_->send(frame);

      static uint64_t count = 0;
      if (++count % 200 == 0) {
        RCLCPP_INFO(this->get_logger(), "已发送 %lu 帧，当前序列号: %lu",
                    count, msg->sequence_id);
      }
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "串口发送失败: %s", e.what());
      // 自动重连
      try {
        serial_port_->close();
        serial_port_->open();  // 重新打开（配置和设备名不变）
        RCLCPP_INFO(this->get_logger(), "串口自动重连成功");
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "串口重连失败，请检查硬件连接");
      }
    }
  }

  // 成员变量
  IoContext io_ctx_;                    // 必须作为第一个成员，保证构造顺序
  std::shared_ptr<SerialPort> serial_port_;
  rclcpp::Subscription<doorlock_sniper::msg::VideoPacket>::SharedPtr video_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoUartSender>());
  rclcpp::shutdown();
  return 0;
}