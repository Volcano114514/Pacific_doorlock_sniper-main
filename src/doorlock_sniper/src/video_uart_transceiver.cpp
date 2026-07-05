#include <cstdint>
#include <vector>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include "doorlock_sniper/msg/video_packet.hpp"
#include "std_msgs/msg/string.hpp"

namespace doorlock_sniper
{

namespace {
  constexpr uint8_t FRAME_HEADER[4] = {0xAA, 0x55, 0xA5, 0x5A};
  constexpr int HEADER_SIZE = 4;
  constexpr int SEQ_SIZE = 2;
  constexpr int PAYLOAD_SIZE = 300;
  constexpr int CRC_SIZE = 1;
  constexpr int FULL_FRAME_SIZE = HEADER_SIZE + SEQ_SIZE + PAYLOAD_SIZE + CRC_SIZE; // 307字节
}

class VideoUartTransceiver : public rclcpp::Node
{
public:
  explicit VideoUartTransceiver(const rclcpp::NodeOptions & options)
  : Node("video_uart_transceiver", options)
  {
    // 参数声明（统一bool类型，launch传字符串true/false可自动解析）
    param_uart_device_ = this->declare_parameter("uart_device", "/dev/ttyACM0");
    param_baudrate_ = this->declare_parameter("baudrate", 921600);
    param_enable_send_ = this->declare_parameter("enable_send", false);
    param_enable_receive_ = this->declare_parameter("enable_receive", true);
    param_debug_mode_ = this->declare_parameter("debug_mode", false);

    // 接收功能：发布解析后的码流 + 格式话题
    if (param_enable_receive_) {
      packet_pub_ = this->create_publisher<doorlock_sniper::msg::VideoPacket>(
        "/video_stream/uart",
        rclcpp::QoS(rclcpp::KeepLast(500)).best_effort());
      format_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/video_format/uart",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
      
      // 发布一次默认格式，避免解码器等待
      std_msgs::msg::String fmt_msg;
      fmt_msg.data = "h264";
      format_pub_->publish(fmt_msg);
    }

    // 发送功能：订阅本地编码流，从串口发出
    if (param_enable_send_) {
      packet_sub_ = this->create_subscription<doorlock_sniper::msg::VideoPacket>(
        "/video_stream",
        rclcpp::QoS(rclcpp::KeepLast(500)).best_effort(),
        std::bind(&VideoUartTransceiver::packet_callback, this, std::placeholders::_1));
    }

    // 初始化串口（同一fd同时读写，无争抢）
    if (!init_serial_port()) {
      RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "串口打开成功 %s @ %d",
      param_uart_device_.c_str(), param_baudrate_);

    if (param_debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "调试模式已开启：原生termios串口轮询");
    }
    RCLCPP_INFO(this->get_logger(), "发送功能: %s, 接收功能: %s",
      param_enable_send_ ? "开启" : "关闭",
      param_enable_receive_ ? "开启" : "关闭");

    // 启动1ms轮询定时器（仅接收开启时启动）
    if (param_enable_receive_) {
      poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&VideoUartTransceiver::serial_poll_callback, this));
    }

    RCLCPP_INFO(this->get_logger(), "节点初始化完成，单帧长度: %d字节", FULL_FRAME_SIZE);
  }

  ~VideoUartTransceiver() override
  {
    if (uart_fd_ >= 0) {
      close(uart_fd_);
      uart_fd_ = -1;
    }
  }

private:
  bool init_serial_port()
  {
    uart_fd_ = open(param_uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", param_uart_device_.c_str());
      return false;
    }

    struct termios tty;
    if (tcgetattr(uart_fd_, &tty) != 0) {
      return false;
    }

    cfmakeraw(&tty);

    // 波特率配置
    speed_t baud = B921600;
    if (param_baudrate_ == 115200) baud = B115200;
    else if (param_baudrate_ == 460800) baud = B460800;
    else if (param_baudrate_ == 921600) baud = B921600;
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1配置
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
      return false;
    }
    tcflush(uart_fd_, TCIOFLUSH);
    return true;
  }

  uint8_t calc_crc8(const uint8_t* data, int len)
  {
    uint8_t crc = 0;
    for (int i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int j = 0; j < 8; ++j) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
      }
    }
    return crc;
  }

  // 发送回调：ROS码流 -> 串口帧
  void packet_callback(const doorlock_sniper::msg::VideoPacket::SharedPtr msg)
  {
    if (uart_fd_ < 0) return;

    uint8_t tx_buf[FULL_FRAME_SIZE];
    // 4字节帧头
    std::memcpy(tx_buf, FRAME_HEADER, HEADER_SIZE);
    // 2字节序号（大端）
    uint16_t seq = static_cast<uint16_t>(msg->sequence_id & 0xFFFF);
    tx_buf[HEADER_SIZE] = static_cast<uint8_t>(seq >> 8);
    tx_buf[HEADER_SIZE + 1] = static_cast<uint8_t>(seq & 0xFF);
    // 300字节载荷
    std::memcpy(tx_buf + HEADER_SIZE + SEQ_SIZE, msg->data.data(), PAYLOAD_SIZE);
    // CRC8校验
    uint8_t crc = calc_crc8(tx_buf, FULL_FRAME_SIZE - CRC_SIZE);
    tx_buf[FULL_FRAME_SIZE - 1] = crc;

    // 串口发送（共用接收的同一个fd）
    ssize_t n = write(uart_fd_, tx_buf, FULL_FRAME_SIZE);
    if (n > 0) {
      tx_frame_cnt_++;
      tx_bytes_total_ += n;
    }
  }

  void serial_poll_callback()
  {
    if (uart_fd_ < 0) return;

    uint8_t read_buf[2048];
    ssize_t n = read(uart_fd_, read_buf, sizeof(read_buf));
    if (n > 0) {
      rx_bytes_total_ += n;
      parse_serial_stream(read_buf, static_cast<size_t>(n));
    }

    // 每秒打印统计
    static int64_t last_stats_ns = 0;
    int64_t now_ns = this->now().nanoseconds();
    if (now_ns - last_stats_ns > 1000000000LL) {
      RCLCPP_INFO(this->get_logger(),
        "收: %lu字节/秒 发: %lu字节/秒 | 有效帧: %lu, 无效假帧: %lu, 发送帧: %lu",
        rx_bytes_total_, tx_bytes_total_,
        valid_frame_cnt_, fake_frame_cnt_, tx_frame_cnt_);
      rx_bytes_total_ = 0;
      tx_bytes_total_ = 0;
      tx_frame_cnt_ = 0;
      last_stats_ns = now_ns;
    }
  }

  void parse_serial_stream(const uint8_t* data, size_t len)
  {
    rx_byte_buffer_.insert(rx_byte_buffer_.end(), data, data + len);

    while (rx_byte_buffer_.size() >= FULL_FRAME_SIZE) {
      const uint8_t* frame_ptr = rx_byte_buffer_.data();

      // 第一重：4字节帧头匹配
      bool header_match = true;
      for (int i = 0; i < HEADER_SIZE; ++i) {
        if (frame_ptr[i] != FRAME_HEADER[i]) {
          header_match = false;
          break;
        }
      }
      if (!header_match) {
        rx_byte_buffer_.erase(rx_byte_buffer_.begin());
        continue;
      }

      // 第二重：CRC8完整性校验
      uint8_t calc_crc = calc_crc8(frame_ptr, FULL_FRAME_SIZE - CRC_SIZE);
      uint8_t recv_crc = frame_ptr[FULL_FRAME_SIZE - 1];
      if (calc_crc != recv_crc) {
        fake_frame_cnt_++;
        rx_byte_buffer_.erase(rx_byte_buffer_.begin());
        continue;
      }

      // 提取序号和载荷
      uint16_t seq = (static_cast<uint16_t>(frame_ptr[HEADER_SIZE]) << 8) | frame_ptr[HEADER_SIZE + 1];
      const uint8_t* payload = frame_ptr + HEADER_SIZE + SEQ_SIZE;

      // 第三重：序号连续性校验（首帧跳过校验，避免误判）
      if (last_valid_seq_ != 0) {
        int32_t seq_diff = static_cast<int32_t>(seq) - static_cast<int32_t>(last_valid_seq_);
        if (seq_diff <= 0 || seq_diff > 200) {
          fake_frame_cnt_++;
          rx_byte_buffer_.erase(rx_byte_buffer_.begin());
          continue;
        }
      }

      // 校验全部通过，发布ROS消息
      auto pkt = doorlock_sniper::msg::VideoPacket();
      pkt.sequence_id = seq;
      pkt.timestamp_ns = this->now().nanoseconds();
      std::memcpy(pkt.data.data(), payload, PAYLOAD_SIZE);
      packet_pub_->publish(pkt);

      valid_frame_cnt_++;
      last_valid_seq_ = seq;

      // 移除已解析的整帧
      rx_byte_buffer_.erase(
        rx_byte_buffer_.begin(),
        rx_byte_buffer_.begin() + FULL_FRAME_SIZE
      );
    }
  }

  // 私有成员
  int uart_fd_ = -1;
  rclcpp::Subscription<doorlock_sniper::msg::VideoPacket>::SharedPtr packet_sub_;
  rclcpp::Publisher<doorlock_sniper::msg::VideoPacket>::SharedPtr packet_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr format_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::vector<uint8_t> rx_byte_buffer_;
  uint16_t last_valid_seq_ = 0;
  uint64_t valid_frame_cnt_ = 0;
  uint64_t fake_frame_cnt_ = 0;
  uint64_t tx_frame_cnt_ = 0;
  uint64_t rx_bytes_total_ = 0;
  uint64_t tx_bytes_total_ = 0;

  std::string param_uart_device_;
  int param_baudrate_;
  bool param_enable_send_;
  bool param_enable_receive_;
  bool param_debug_mode_;
};

} // namespace doorlock_sniper

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<doorlock_sniper::VideoUartTransceiver>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
