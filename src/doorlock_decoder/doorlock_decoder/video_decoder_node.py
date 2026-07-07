#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket, VideoPacketVar
from std_msgs.msg import String
from sensor_msgs.msg import Image
import av
import cv2
import threading
import queue
import time
import numpy as np
from pathlib import Path
from cv_bridge import CvBridge


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')

        # ========== 参数声明 ==========
        self.declare_parameter('topic', '/video_stream/shark')
        self.declare_parameter('format_topic', '/video_format/shark')
        self.declare_parameter('use_variable_packet', False)
        self.declare_parameter('display', True)
        self.declare_parameter('window_title', 'Doorlock Decoder')
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_save_decoder', True)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')
        self.declare_parameter('publish_image', True)
        self.declare_parameter('output_topic', 'decoded_image')   # 新增
        self.declare_parameter('default_codec', 'hevc')
        self.declare_parameter('disable_auto_switch', True)
        self.declare_parameter('reset_on_gap', False)
        self.declare_parameter('gap_reset_threshold', 50)
        self.declare_parameter('decode_error_reset_threshold', 80)

        # ========== 参数读取 ==========
        topic = self.get_parameter('topic').value
        format_topic = self.get_parameter('format_topic').value
        self.use_var_packet = self.get_parameter('use_variable_packet').value
        self.display = self.get_parameter('display').value
        self.window_title = str(self.get_parameter('window_title').value)
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
        self.display_frame_counter = 0
        self.publish_image = bool(self.get_parameter('publish_image').value)
        self.output_topic = self.get_parameter('output_topic').value   # 新增
        self.last_frame_time = 0.0
        self.stream_timeout = 3.0

        self.reset_on_gap = bool(self.get_parameter('reset_on_gap').value)
        self.gap_reset_threshold = int(self.get_parameter('gap_reset_threshold').value)
        self.decode_error_threshold = int(self.get_parameter('decode_error_reset_threshold').value)

        if self.debug_dump_enable and self.debug_dump_save_decoder:
            self.debug_dump_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Debug dump enabled: every {self.debug_dump_every_n_frames} frames -> {self.debug_dump_dir}'
            )

        if self.publish_image:
            self.cv_bridge = CvBridge()
            self.img_pub = self.create_publisher(
                Image,
                self.output_topic,   # 使用可配置的输出话题
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
            )

        self.current_format = str(self.get_parameter('default_codec').value).lower()
        self.disable_auto_switch = bool(self.get_parameter('disable_auto_switch').value)
        self.codec = None
        self._reset_decoder(log=False, reason='startup')

        self.frame_count = 0
        self.packet_count = 0
        self.gap_count = 0
        self.last_seq = None
        self._last_switch_time = 0
        self._consecutive_decode_errors = 0

        self.format_sub = self.create_subscription(
            String,
            format_topic,
            self._format_change_callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        )

        if self.display:
            self.frame_queue = queue.Queue(maxsize=5)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

        # 大队列深度，减少高码率丢包
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=500
        )

        # 根据参数选择订阅定长/变长消息
        packet_type = VideoPacketVar if self.use_var_packet else VideoPacket
        self.subscription = self.create_subscription(
            packet_type,
            topic,
            self._packet_callback,
            qos
        )

        self.get_logger().info(f'Decoder started: subscribing to {topic}')
        self.get_logger().info(f'Packet mode: {"variable length" if self.use_var_packet else "fixed 300 bytes"}')
        self.get_logger().info(f'Default codec: {self.current_format.upper()}, auto switch: {"off" if self.disable_auto_switch else "on"}')
        self.get_logger().info(f'Reset on gap: {"on" if self.reset_on_gap else "off"}, error threshold: {self.decode_error_threshold}')
        if self.publish_image:
            self.get_logger().info(f'Decoded image publisher enabled on topic: {self.output_topic}')

    def _reset_decoder(self, *, log: bool = True, reason: str = '', codec: str = None):
        if codec is None:
            codec = self.current_format

        self.codec = av.CodecContext.create(codec, 'r')
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

        self._consecutive_decode_errors = 0
        if log:
            self.get_logger().warn(f'🔄 Reset decoder to {codec.upper()} ({reason})')

    def _format_change_callback(self, msg):
        new_format = msg.data.lower()
        if new_format not in ['h264', 'hevc']:
            return
        if new_format != self.current_format:
            self.current_format = new_format
            self._reset_decoder(reason='format notification', codec=new_format)

    def _handle_decoded_frame(self, frame):
        if frame is None or frame.width == 0 or frame.height == 0:
            return
        self._consecutive_decode_errors = 0

        img = frame.to_ndarray(format='bgr24').copy()
        if img is None or img.size == 0:
            return
        self.frame_count += 1
        self.last_frame_time = time.time()

        if self.publish_image:
            try:
                img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.img_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().debug(f'Publish image failed: {e}')

        if self.display:
            try:
                self.frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self.frame_count % 60 == 0:
            self.get_logger().info(f'Decoded {self.frame_count} frames')

    def _packet_callback(self, msg):
        self.packet_count += 1

        if self.last_seq is not None and msg.sequence_id != self.last_seq + 1:
            lost_packets = msg.sequence_id - self.last_seq - 1
            self.gap_count += 1
            if lost_packets >= 20:
                self.get_logger().debug(f'Large gap: lost {lost_packets} packets')
                if self.reset_on_gap and lost_packets >= self.gap_reset_threshold:
                    self._reset_decoder(reason='large sequence gap')

        self.last_seq = msg.sequence_id
        chunk = bytes(msg.data)

        try:
            parsed_packets = self.codec.parse(chunk)
            for packet in parsed_packets:
                for frame in self.codec.decode(packet):
                    self._handle_decoded_frame(frame)

        except av.AVError:
            self._consecutive_decode_errors += 1
            if self._consecutive_decode_errors >= self.decode_error_threshold:
                self.get_logger().warn(f'🔄 Consecutive decode errors ({self._consecutive_decode_errors}), reset decoder')
                self._reset_decoder(reason='consecutive decode errors')

            if time.time() - self._last_switch_time > 5.0 and not self.disable_auto_switch:
                self._last_switch_time = time.time()
                self.current_format = 'h264' if self.current_format == 'hevc' else 'hevc'
                self._reset_decoder(reason='decode error auto-switch', codec=self.current_format)

        if self.packet_count % 600 == 0:
            self.get_logger().info(
                f'📊 Rx pkts={self.packet_count}, frames={self.frame_count}, '
                f'gaps={self.gap_count}, format={self.current_format.upper()}'
            )

    def _display_loop(self):
        cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_title, self.display_width, self.display_height)

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

            cv2.imshow(self.window_title, img_disp)

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