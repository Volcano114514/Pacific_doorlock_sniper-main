import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket
import av
import av.error
import cv2
import threading
import queue
import time
import numpy as np
from pathlib import Path


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')

        # ========== 参数声明 ==========
        self.declare_parameter('topic', '/video_stream')
        self.declare_parameter('display', True)
        self.declare_parameter('width', 300)
        self.declare_parameter('height', 300)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_save_decoder', True)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')
        self.declare_parameter('error_reset_threshold', 30)
        self.declare_parameter('gap_reset_threshold', 50)  # 丢包超过该数量才重置

        # ========== 参数读取 ==========
        topic = self.get_parameter('topic').value
        self.display = self.get_parameter('display').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.display_scale = max(1, int(self.get_parameter('display_scale').value))
        self.display_width = self.width * self.display_scale
        self.display_height = self.height * self.display_scale
        self.crosshair_offset_x = int(self.get_parameter('crosshair_offset_x').value)
        self.crosshair_offset_y = int(self.get_parameter('crosshair_offset_y').value)
        self.crosshair_width = max(1, int(self.get_parameter('crosshair_width').value))
        self.debug_dump_enable = bool(self.get_parameter('debug_dump_enable').value)
        self.debug_dump_every_n_frames = max(1, int(self.get_parameter('debug_dump_every_n_frames').value))
        self.debug_dump_save_decoder = bool(self.get_parameter('debug_dump_save_decoder').value)
        self.debug_dump_dir = Path(str(self.get_parameter('debug_dump_dir').value)) / 'decoder'
        self.error_reset_threshold = int(self.get_parameter('error_reset_threshold').value)
        self.gap_reset_threshold = int(self.get_parameter('gap_reset_threshold').value)
        self.display_frame_counter = 0
        self.last_frame_time = 0.0
        self.stream_timeout = 3.0

        if self.debug_dump_enable and self.debug_dump_save_decoder:
            self.debug_dump_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Debug dump enabled: every {self.debug_dump_every_n_frames} frames -> {self.debug_dump_dir}'
            )

        # ========== 解码器初始化 ==========
        self.codec = None
        self._consecutive_errors = 0
        self._total_lost_packets = 0
        self._reset_decoder(log=False, reason='startup')

        self.frame_count = 0
        self.packet_count = 0
        self.parsed_packet_count = 0
        self.gap_count = 0
        self.last_seq = None

        # ========== 显示线程 ==========
        if self.display:
            self.frame_queue = queue.Queue(maxsize=5)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

        # ========== 订阅配置 ==========
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=500
        )
        self.subscription = self.create_subscription(
            VideoPacket,
            topic,
            self._packet_callback,
            qos
        )

        self.get_logger().info(f'Decoder started: subscribing to {topic}')
        self.get_logger().info(f'Packet mode: fixed 300 bytes')
        self.get_logger().info(f'Default codec: H264')
        self.get_logger().info(f'Gap reset threshold: {self.gap_reset_threshold} packets')

    def _reset_decoder(self, *, log: bool = True, reason: str = ''):
        """重置解码器上下文，清零错误计数"""
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'FRAME'
        try:
            self.codec.flags |= av.codec.context.Flags.LOW_DELAY
            self.codec.flags2 |= av.codec.context.Flags2.FAST
            self.codec.options = {
                'err_detect': 'ignore_err',
                'flags2': 'showall'
            }
        except (AttributeError, ValueError):
            pass
        self._consecutive_errors = 0
        self._total_lost_packets = 0
        if log:
            self.get_logger().warn(f'Reset decoder ({reason})')

    def _handle_decoded_frame(self, frame):
        """处理解码成功的帧"""
        if frame is None or frame.width == 0 or frame.height == 0:
            return
        self._consecutive_errors = 0
        self._total_lost_packets = 0

        img = frame.to_ndarray(format='bgr24').copy()
        if img is None or img.size == 0:
            return
        self.frame_count += 1
        self.last_frame_time = time.time()

        if self.display:
            try:
                self.frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self.frame_count % 60 == 0:
            self.get_logger().info(f'Decoded {self.frame_count} frames')

    def _packet_callback(self, msg):
        """接收分包，拼接解码"""
        self.packet_count += 1
        current_seq = msg.sequence_id

        # ========== 异常包过滤：假包序列号通常跳变极大或回退 ==========
        if self.last_seq is not None:
            seq_diff = current_seq - self.last_seq
            # 序列号回退 或 跳变超过1000：判定为假包，直接丢弃，不更新序号
            if seq_diff <= 0 or seq_diff > 1000:
                self.gap_count += 1
                return  # 直接丢包，不送入解码，也不打乱last_seq

            # 正常丢包：累计计数，超过阈值才重置
            if seq_diff > 1:
                lost = seq_diff - 1
                self._total_lost_packets += lost
                self.gap_count += 1

                if self._total_lost_packets >= self.gap_reset_threshold:
                    self.get_logger().warn(
                        f'Cumulative lost {self._total_lost_packets} packets, reset decoder'
                    )
                    self._reset_decoder(reason='cumulative packet loss')
                    self.last_seq = current_seq
                    return

        self.last_seq = current_seq
        chunk = bytes(msg.data)

        try:
            parsed_packets = self.codec.parse(chunk)
            self.parsed_packet_count += len(parsed_packets)
            for packet in parsed_packets:
                for frame in self.codec.decode(packet):
                    self._handle_decoded_frame(frame)

        except av.error.InvalidDataError:
            # 无效数据坏包，最常见
            self._consecutive_errors += 1
            if self._consecutive_errors >= self.error_reset_threshold:
                self.get_logger().warn(
                    f'Consecutive invalid data errors ({self._consecutive_errors}), reset decoder'
                )
                self._reset_decoder(reason='consecutive invalid data')

        except av.error.AVError as e:
            # 其他解码错误兜底
            self._consecutive_errors += 1
            self.get_logger().debug(f'Decode AVError: {e!s}')
            if self._consecutive_errors >= self.error_reset_threshold:
                self.get_logger().warn(
                    f'Consecutive decode errors ({self._consecutive_errors}), reset decoder'
                )
                self._reset_decoder(reason='consecutive decode errors')

        if self.packet_count % 600 == 0:
            self.get_logger().info(
                f'Rx pkts={self.packet_count}, parsed_h264={self.parsed_packet_count}, '
                f'frames={self.frame_count}, gaps={self.gap_count}'
            )

    def _display_loop(self):
        cv2.namedWindow('Doorlock Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Doorlock Decoder', self.display_width, self.display_height)

        last_img = None

        while rclpy.ok():
            current_time = time.time()
            has_stream = (self.last_frame_time > 0) and (current_time - self.last_frame_time < self.stream_timeout)

            try:
                img = self.frame_queue.get(timeout=0.03)
                if img is None:
                    break
                last_img = img
            except queue.Empty:
                pass

            if has_stream and last_img is not None:
                img_disp = cv2.resize(
                    last_img,
                    (self.display_width, self.display_height),
                    interpolation=cv2.INTER_LINEAR
                )
                self._draw_overlay(img_disp)

                if self.debug_dump_enable and self.debug_dump_save_decoder:
                    self.display_frame_counter += 1
                    if self.display_frame_counter % self.debug_dump_every_n_frames == 0:
                        frame_id = f'{self.display_frame_counter:08d}'
                        out_path = self.debug_dump_dir / f'decoder_{frame_id}.png'
                        cv2.imwrite(str(out_path), img_disp)
            else:
                img_disp = np.full((self.display_height, self.display_width, 3), 34, dtype=np.uint8)
                text = "Waiting for video stream..."
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                thickness = 2
                (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
                text_x = (self.display_width - text_w) // 2
                text_y = (self.display_height + text_h) // 2
                cv2.putText(img_disp, text, (text_x, text_y),
                            font, font_scale, (200, 200, 200), thickness, cv2.LINE_AA)

            cv2.imshow('Doorlock Decoder', img_disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, w // 2 + self.crosshair_offset_x))
        cy = max(0, min(h - 1, h // 2 + self.crosshair_offset_y))
        crosshair_color = (230, 190, 235)
        cv2.line(img, (0, cy), (w - 1, cy), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        center_color = (170, 255, 170)
        center = (w // 2, h // 2)
        cv2.circle(img, center, 24, center_color, 1, cv2.LINE_AA)

    def destroy_node(self):
        if self.display:
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self.display_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoderNode()
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
