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

# ==================== 官方协议固定常量 ====================
UDP_MAX_PACKET_SIZE = 1400
UDP_HEADER_SIZE = 8
UDP_MAX_PAYLOAD = 1392
# =========================================================

class SharkToROS2Bridge(Node):
    def __init__(self):
        super().__init__('shark_to_ros2_bridge')
        
        self.declare_parameter('shark_ip', '192.168.12.1')
        self.declare_parameter('shark_port', 3334)
        self.declare_parameter('packet_size', 300)
        self.declare_parameter('frame_timeout_s', 1.5)
        
        self.shark_ip = self.get_parameter('shark_ip').value
        self.shark_port = self.get_parameter('shark_port').value
        self.packet_size = self.get_parameter('packet_size').value
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
        
        # 高码率适配：增大发布队列深度
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=500
        )
        # 使用变长消息发布UDP流
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
        
        if len(data) > UDP_MAX_PACKET_SIZE:
            self.get_logger().warn(f'Invalid packet size: {len(data)} bytes (max 1400), dropped')
            return
        if len(data) < UDP_HEADER_SIZE:
            return
        
        frame_id = struct.unpack('!H', data[0:2])[0]
        chunk_idx = struct.unpack('!H', data[2:4])[0]
        total_bytes = struct.unpack('!I', data[4:8])[0]
        payload = data[8:]
        
        if total_bytes == 0 or total_bytes > 2*1024*1024:
            return
        max_chunk = (total_bytes + UDP_MAX_PAYLOAD - 1) // UDP_MAX_PAYLOAD
        if chunk_idx >= max_chunk:
            return
        
        if self.current_format is None:
            fmt = self._detect_format(payload)
            if fmt:
                self.current_format = fmt
                self.get_logger().info(f'✅ Auto-detected format: {fmt.upper()}')
                msg = String()
                msg.data = fmt
                self.format_pub.publish(msg)
        
        # 帧ID回绕处理
        if len(self.frame_buffer) > 0:
            existing_ids = list(self.frame_buffer.keys())
            for eid in existing_ids:
                if frame_id < 1000 and eid > 50000:
                    del self.frame_buffer[eid]
                    self.lost_frame_count += 1
        
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
        
        if chunk_idx in frame['chunks']:
            return
        
        frame['chunks'][chunk_idx] = payload
        frame['received'] += len(payload)
        
        # 收齐所有分片才发布，避免残缺帧
        if len(frame['chunks']) == frame['max_chunk'] and frame['received'] >= frame['total']:
            sorted_keys = sorted(frame['chunks'].keys())
            complete_frame = b''.join([frame['chunks'][k] for k in sorted_keys])
            
            if len(complete_frame) != frame['total']:
                self.error_frame_count += 1
                del self.frame_buffer[frame_id]
                return
            
            self._publish_frame(complete_frame)
            self.complete_frame_count += 1
            del self.frame_buffer[frame_id]
    
    def _publish_frame(self, frame_data):
        chunk_size = self.packet_size
        for i in range(0, len(frame_data), chunk_size):
            chunk = frame_data[i:i+chunk_size]
            # 核心改进：变长消息，无需补0，直接发送原始码流
            msg = VideoPacketVar()
            msg.sequence_id = self.sequence_id
            msg.timestamp_ns = self.get_clock().now().nanoseconds
            msg.data = array('B', chunk)
            self.packet_pub.publish(msg)
            self.sequence_id += 1
    
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
