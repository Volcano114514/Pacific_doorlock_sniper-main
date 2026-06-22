import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket
from std_msgs.msg import String
import socket
import struct
import threading
import time
from collections import deque

class SharkToROS2Bridge(Node):
    def __init__(self):
        super().__init__('shark_to_ros2_bridge')
        
        # 参数配置
        self.declare_parameter('shark_ip', '192.168.100.1')
        self.declare_parameter('shark_port', 3334)
        self.declare_parameter('packet_size', 300)
        self.declare_parameter('frame_timeout_s', 1.0)
        
        self.shark_ip = self.get_parameter('shark_ip').value
        self.shark_port = self.get_parameter('shark_port').value
        self.packet_size = self.get_parameter('packet_size').value
        self.frame_timeout_s = self.get_parameter('frame_timeout_s').value
        
        # 帧缓存
        self.frame_buffer = {}
        self.sequence_id = 0
        
        # 视频格式检测与发布
        self.current_format = None
        self.format_pub = self.create_publisher(
            String, 
            '/video_format', 
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=300
        )
        
        self.packet_pub = self.create_publisher(VideoPacket, '/video_stream', qos)
        
        # 创建UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(0.1)
        self.udp_socket.bind(('0.0.0.0', self.shark_port))
        
        # 启动线程
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
        self.receive_thread.start()
        self.cleanup_thread.start()
        
        self.get_logger().info(f'SharkDataServer bridge started: listening on 0.0.0.0:{self.shark_port}')
        self.get_logger().info(f'Forwarding to ROS2 topic: /video_stream')
        self.get_logger().info('Auto video format detection enabled')
    
    def _detect_video_format(self, data_chunk):
        """自动检测Annex-B格式的视频流编码类型"""
        start_codes = [b'\x00\x00\x01', b'\x00\x00\x00\x01']
        
        for start_code in start_codes:
            pos = data_chunk.find(start_code)
            if pos == -1:
                continue
                
            nal_header_pos = pos + len(start_code)
            if nal_header_pos >= len(data_chunk):
                continue
                
            nal_header = data_chunk[nal_header_pos]
            
            # HEVC检测（NAL类型占高6位）
            hevc_nal_type = (nal_header >> 1) & 0x3F
            if hevc_nal_type in [32, 33, 34]:  # VPS, SPS, PPS
                return 'hevc'
                
            # H.264检测（NAL类型占低5位）
            h264_nal_type = nal_header & 0x1F
            if h264_nal_type in [7, 8]:  # SPS, PPS
                return 'h264'
                
        return None
    
    def _process_udp_packet(self, data):
        # 解析Shark UDP包头（大端序）
        frame_id = struct.unpack('!H', data[0:2])[0]
        chunk_index = struct.unpack('!H', data[2:4])[0]
        total_bytes = struct.unpack('!I', data[4:8])[0]
        video_data = data[8:]
        
        # 自动检测视频格式
        if self.current_format is None:
            detected_format = self._detect_video_format(video_data)
            if detected_format is not None:
                self.current_format = detected_format
                self.get_logger().info(f'✅ Auto-detected video format: {detected_format.upper()}')
                format_msg = String()
                format_msg.data = detected_format
                self.format_pub.publish(format_msg)
        
        # 初始化帧缓存
        if frame_id not in self.frame_buffer:
            self.frame_buffer[frame_id] = {
                'chunks': [],
                'total_bytes': total_bytes,
                'received_bytes': 0,
                'timestamp': time.time()
            }
            self.frame_buffer[frame_id]['chunks'] = [None] * ((total_bytes + 1023) // 1024)
        
        frame = self.frame_buffer[frame_id]
        
        # 存储分片
        if frame['chunks'][chunk_index] is None:
            frame['chunks'][chunk_index] = video_data
            frame['received_bytes'] += len(video_data)
        
        # 检查是否接收完整
        if frame['received_bytes'] == frame['total_bytes']:
            complete_frame = b''.join(frame['chunks'])
            self._publish_frame(complete_frame)
            del self.frame_buffer[frame_id]
    
    def _publish_frame(self, frame_data):
        # 将完整帧分片为300字节的VideoPacket
        for i in range(0, len(frame_data), self.packet_size):
            chunk = frame_data[i:i+self.packet_size]
            msg = VideoPacket()
            msg.sequence_id = self.sequence_id
            msg.timestamp_ns = self.get_clock().now().nanoseconds
            msg.data[:len(chunk)] = chunk
            self.packet_pub.publish(msg)
            self.sequence_id += 1
        
        if self.sequence_id % 600 == 0:
            self.get_logger().info(f'Forwarded {self.sequence_id} packets')
    
    def _receive_loop(self):
        """UDP接收循环（独立线程）"""
        while self.running and rclpy.ok():
            try:
                data, addr = self.udp_socket.recvfrom(1032)
                self._process_udp_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')
    
    def _cleanup_loop(self):
        """定期清理超时的不完整帧"""
        while self.running and rclpy.ok():
            now = time.time()
            expired_frames = []
            for frame_id, frame in self.frame_buffer.items():
                if now - frame['timestamp'] > self.frame_timeout_s:
                    expired_frames.append(frame_id)
            
            for frame_id in expired_frames:
                self.get_logger().warn(f'Frame {frame_id} timed out, dropping')
                del self.frame_buffer[frame_id]
            
            time.sleep(0.5)
    
    def destroy_node(self):
        self.running = False
        self.receive_thread.join(timeout=1.0)
        self.cleanup_thread.join(timeout=1.0)
        self.udp_socket.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SharkToROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
