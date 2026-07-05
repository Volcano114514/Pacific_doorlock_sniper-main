import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacketVar
from std_msgs.msg import String
import socket
import struct
import threading
import time
from array import array

# ==================== 官方UDP图传协议常量（完全对齐） ====================
UDP_MAX_PACKET_SIZE = 1400    # 单UDP包总长度
UDP_HEADER_SIZE = 8           # 包头长度（frame_id + chunk_idx + total_bytes）
UDP_MAX_PAYLOAD = 1392        # 单包有效码流载荷 = 1400 - 8
# =======================================================================

class SharkToROS2Bridge(Node):
    def __init__(self):
        super().__init__('shark_to_ros2_bridge')
        
        self.declare_parameter('shark_ip', '192.168.12.1')
        self.declare_parameter('shark_port', 3334)
        self.declare_parameter('frame_timeout_s', 1.5)
        
        self.shark_ip = self.get_parameter('shark_ip').value
        self.shark_port = self.get_parameter('shark_port').value
        self.frame_timeout_s = self.get_parameter('frame_timeout_s').value
        
        self.recv_packet_count = 0
        self.complete_frame_count = 0
        self.lost_frame_count = 0
        self.error_frame_count = 0
        self.frame_buffer = {}
        self.sequence_id = 0
        
        self.current_format = None
        self.format_pub = self.create_publisher(
            String, '/video_format/shark',
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )
        
        # 高码率适配：增大发布队列深度，减少ROS层面丢包
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=500
        )
        # 使用变长消息发布UDP流，单包大小与原始UDP载荷对齐，无填充
        self.packet_pub = self.create_publisher(VideoPacketVar, '/video_stream/shark', qos)
        
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024)
        self.recv_sock.settimeout(0.1)
        self.recv_sock.bind(('0.0.0.0', self.shark_port))
        
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
        self.receive_thread.start()
        self.cleanup_thread.start()
        
        try:
            send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            send_sock.sendto(b'HELLO', (self.shark_ip, self.shark_port))
            send_sock.close()
            self.get_logger().info(f'Triggered stream from {self.shark_ip}:{self.shark_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to trigger stream: {type(e).__name__}: {e}')
        
        self.create_timer(2.0, self._print_stats)
    
    def _print_stats(self):
        self.get_logger().info(
            f'Stats: {self.recv_packet_count} pkts, {self.complete_frame_count} frames, '
            f'lost: {self.lost_frame_count}, error: {self.error_frame_count}'
        )
    
    def _detect_format(self, data):
        start_codes = [b'\x00\x00\x01', b'\x00\x00\x00\x01']
        for sc in start_codes:
            pos = data.find(sc)
            if pos == -1:
                continue
            nal_pos = pos + len(sc)
            if nal_pos >= len(data):
                continue
            nal_byte = data[nal_pos]
            hevc_type = (nal_byte >> 1) & 0x3F
            if hevc_type in [32, 33, 34]:
                return 'hevc'
            avc_type = nal_byte & 0x1F
            if avc_type in [7, 8]:
                return 'h264'
        return None
    
    def _process_packet(self, data):
        self.recv_packet_count += 1
        
        # 包长度合法性校验
        if len(data) > UDP_MAX_PACKET_SIZE or len(data) < UDP_HEADER_SIZE:
            self.get_logger().warn(f'Invalid packet size: {len(data)} bytes, dropped')
            return
        
        # 解析包头（完全对齐官方协议）
        frame_id = struct.unpack('!H', data[0:2])[0]
        chunk_idx = struct.unpack('!H', data[2:4])[0]
        total_bytes = struct.unpack('!I', data[4:8])[0]
        payload = data[8:]
        
        # 异常值过滤
        if total_bytes == 0 or total_bytes > 2*1024*1024:
            return
        # 计算总分片数，按1392字节/包向上取整
        max_chunk = (total_bytes + UDP_MAX_PAYLOAD - 1) // UDP_MAX_PAYLOAD
        if chunk_idx >= max_chunk:
            return
        
        # 首次接收到码流时自动识别编码格式
        if self.current_format is None:
            fmt = self._detect_format(payload)
            if fmt:
                self.current_format = fmt
                self.get_logger().info(f'✅ Auto-detected format: {fmt.upper()}')
                msg = String()
                msg.data = fmt
                self.format_pub.publish(msg)
        
        # 帧ID回绕处理（uint16溢出后重置）
        if len(self.frame_buffer) > 0:
            existing_ids = list(self.frame_buffer.keys())
            for eid in existing_ids:
                if frame_id < 1000 and eid > 50000:
                    del self.frame_buffer[eid]
                    self.lost_frame_count += 1
        
        # 初始化帧缓冲区
        if frame_id not in self.frame_buffer:
            self.frame_buffer[frame_id] = {
                'chunks': {},
                'total': total_bytes,
                'max_chunk': max_chunk,
                'received': 0,
                'ts': time.time()
            }
        
        frame = self.frame_buffer[frame_id]
        frame['ts'] = time.time()
        
        # 重复分片直接丢弃
        if chunk_idx in frame['chunks']:
            return
        
        frame['chunks'][chunk_idx] = payload
        frame['received'] += len(payload)
        
        # 收齐全部分片后，按序号顺序直接发布原始载荷
        if len(frame['chunks']) == frame['max_chunk']:
            # 校验总长度是否匹配
            if frame['received'] != frame['total']:
                self.error_frame_count += 1
                del self.frame_buffer[frame_id]
                return
            
            # 按分片序号从小到大依次发布，保证码流顺序正确
            for idx in sorted(frame['chunks'].keys()):
                chunk_data = frame['chunks'][idx]
                msg = VideoPacketVar()
                msg.sequence_id = self.sequence_id
                msg.timestamp_ns = self.get_clock().now().nanoseconds
                msg.data = array('B', chunk_data)
                self.packet_pub.publish(msg)
                self.sequence_id += 1
            
            self.complete_frame_count += 1
            del self.frame_buffer[frame_id]
    
    def _receive_loop(self):
        while self.running and rclpy.ok():
            try:
                data, _ = self.recv_sock.recvfrom(2048)
                self._process_packet(data)
            except socket.timeout:
                continue
            except struct.error:
                continue
            except Exception as e:
                import traceback
                self.get_logger().error(f'Receive error: {type(e).__name__}: {e}')
                self.get_logger().error(traceback.format_exc())
    
    def _cleanup_loop(self):
        while self.running and rclpy.ok():
            now = time.time()
            expired = [fid for fid, f in self.frame_buffer.items() 
                       if now - f['ts'] > self.frame_timeout_s]
            for fid in expired:
                self.lost_frame_count += 1
                del self.frame_buffer[fid]
            if len(expired) > 0:
                self.get_logger().warn(f'{len(expired)} frames timed out, total lost: {self.lost_frame_count}')
            time.sleep(0.5)
    
    def destroy_node(self):
        self.running = False
        self.receive_thread.join(timeout=1.0)
        self.cleanup_thread.join(timeout=1.0)
        self.recv_sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SharkToROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
