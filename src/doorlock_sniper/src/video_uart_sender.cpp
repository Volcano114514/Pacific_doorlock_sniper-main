#include <rclcpp/rclcpp.hpp>
#include <doorlock_sniper/msg/video_packet.hpp>
#include <serial_driver/serial_port.hpp>
#include <memory>
#include <cstring>

using namespace drivers::serial_driver;
using namespace drivers::common;

// ========== 新版协议常量（与接收端严格一致） ==========
constexpr uint8_t FRAME_HEADER[4] = {0xAA, 0x55, 0xA5, 0x5A};
constexpr size_t HEADER_SIZE = 4;
constexpr size_t SEQ_SIZE = 2;
constexpr size_t PAYLOAD_SIZE = 300;
constexpr size_t CRC_SIZE = 1;
constexpr size_t TOTAL_FRAME_SIZE = HEADER_SIZE + SEQ_SIZE + PAYLOAD_SIZE + CRC_SIZE; // 307字节

/**
 * @brief CRC8计算（多项式0x07，与接收端算法完全匹配）
 */
static uint8_t calc_crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for(size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

class VideoUartSender : public rclcpp::Node
{
public:
  VideoUartSender()
  : Node("video_uart_sender"),
    io_ctx_()
  {
    // 参数声明与读取
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
      serial_port_ = std::make_shared<SerialPort>(io_ctx_, serial_port, config);
      serial_port_->open();
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

    // 订阅视频编码流
    video_sub_ = this->create_subscription<doorlock_sniper::msg::VideoPacket>(
      "video_stream",
      rclcpp::QoS(rclcpp::KeepLast(3000)).reliable(),
      std::bind(&VideoUartSender::video_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "视频UART发送节点已启动，单帧长度: %zu字节", TOTAL_FRAME_SIZE);
    RCLCPP_INFO(this->get_logger(), "协议格式: 4字节帧头 + 2字节序号 + 300字节数据 + CRC8校验");
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

      // 1. 4字节帧头
      std::memcpy(frame.data() + offset, FRAME_HEADER, HEADER_SIZE);
      offset += HEADER_SIZE;

      // 2. 2字节序号（大端模式，取sequence_id低16位）
      uint16_t seq = static_cast<uint16_t>(msg->sequence_id & 0xFFFF);
      frame[offset++] = static_cast<uint8_t>(seq >> 8);
      frame[offset++] = static_cast<uint8_t>(seq & 0xFF);

      // 3. 300字节视频载荷（原封不动）
      std::memcpy(frame.data() + offset, msg->data.data(), PAYLOAD_SIZE);
      offset += PAYLOAD_SIZE;

      // 4. CRC8校验（校验帧头+序号+载荷）
      uint8_t crc = calc_crc8(frame.data(), offset);
      frame[offset++] = crc;

      // 5. 串口发送
      serial_port_->send(frame);

      static uint64_t count = 0;
      if (++count % 200 == 0) {
        RCLCPP_INFO(this->get_logger(), "已发送 %lu 帧，当前序号: %u",
                    count, seq);
      }

    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "串口发送失败: %s", e.what());
      // 自动重连
      try {
        serial_port_->close();
        serial_port_->open();
        RCLCPP_INFO(this->get_logger(), "串口自动重连成功");
      } catch (...) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "串口重连失败，请检查硬件连接");
      }
    }
  }

  // 成员变量
  IoContext io_ctx_;
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
