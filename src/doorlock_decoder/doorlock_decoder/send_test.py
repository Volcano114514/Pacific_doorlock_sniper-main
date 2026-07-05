import rclpy
from rclpy.node import Node
from doorlock_sniper.msg import VideoPacket

def main():
    rclpy.init()
    node = Node("uart_test_pub")
    pub = node.create_publisher(VideoPacket, "/video_stream", 10)
    msg = VideoPacket()
    msg.sequence_id = 888
    msg.timestamp_ns = 0
    msg.data = [0] * 300
    msg.data[0] = 1
    msg.data[1] = 2
    msg.data[2] = 3
    msg.data[3] = 4
    msg.data[4] = 5
    pub.publish(msg)
    node.get_logger().info("已下发seq=888测试帧")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
