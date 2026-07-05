import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from doorlock_sniper.msg import VideoPacket

class EchoChecker(Node):
    def __init__(self):
        super().__init__("echo_checker")
        # 与发布端对齐 QoS：best_effort + depth=10
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            VideoPacket,
            "/video_stream/uart",
            self.callback,
            qos
        )
        self.found = False
        self.get_logger().info("正在监听 seq=888 的回显帧...")

    def callback(self, msg):
        if msg.sequence_id == 888 and not self.found:
            self.found = True
            self.get_logger().info("✅ 捕获到回显帧！seq=888")
            self.get_logger().info(f"前5字节数据: {list(msg.data[:5])}")
            # 预期前5字节十进制：17 34 51 68 85 (对应 0x11~0x55)

def main():
    rclpy.init()
    node = EchoChecker()
    start_time = node.get_clock().now()

    # 最多监听10秒，超时自动退出
    while rclpy.ok() and not node.found:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed_ns = (node.get_clock().now() - start_time).nanoseconds
        if elapsed_ns / 1e9 > 10:
            node.get_logger().warn("❌ 10秒内未收到回显帧，下行链路异常")
            break

    rclpy.shutdown()

if __name__ == "__main__":
    main()
