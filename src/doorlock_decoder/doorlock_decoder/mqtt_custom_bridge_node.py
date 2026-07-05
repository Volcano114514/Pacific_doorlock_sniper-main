import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from doorlock_sniper.msg import VideoPacket
from std_msgs.msg import String
import struct
import threading
import time
from array import array
import paho.mqtt.client as mqtt
from .rm_referee_pb2 import *

class FullRefereeBridge(Node):
    def __init__(self):
        super().__init__('full_referee_bridge')
        
        self.declare_parameter('mqtt_server_ip', '192.168.12.1')
        self.declare_parameter('mqtt_server_port', 3333)
        self.declare_parameter('robot_id', 3)
        self.declare_parameter('packet_size', 300)
        self.declare_parameter('frame_timeout_s', 1.5)
        self.declare_parameter('enable_video_stream', True)
        
        self.mqtt_ip = self.get_parameter('mqtt_server_ip').value
        self.mqtt_port = self.get_parameter('mqtt_server_port').value
        self.robot_id = self.get_parameter('robot_id').value
        self.packet_size = self.get_parameter('packet_size').value
        self.frame_timeout_s = self.get_parameter('frame_timeout_s').value
        self.enable_video = self.get_parameter('enable_video_stream').value
        
        self.recv_block_count = 0
        self.complete_frame_count = 0
        self.lost_frame_count = 0
        self.frame_buffer = {}
        self.sequence_id = 0
        self.current_format = None
        self.raw_pubs = {}
        
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=500
        )
        
        self.packet_pub = self.create_publisher(VideoPacket, '/video_stream/mqtt', qos_best_effort)
        self.format_pub = self.create_publisher(String, '/video_format/mqtt', qos_reliable)
        
        # 裁判系统话题发布
        self.game_status_pub = self.create_publisher(String, '/referee/game_status', qos_reliable)
        self.global_unit_pub = self.create_publisher(String, '/referee/global_unit', qos_reliable)
        self.global_logistics_pub = self.create_publisher(String, '/referee/global_logistics', qos_reliable)
        self.global_mechanism_pub = self.create_publisher(String, '/referee/global_mechanism', qos_reliable)
        self.event_pub = self.create_publisher(String, '/referee/event', qos_reliable)
        self.robot_injury_pub = self.create_publisher(String, '/referee/robot_injury', qos_reliable)
        self.robot_respawn_pub = self.create_publisher(String, '/referee/robot_respawn', qos_reliable)
        self.robot_dynamic_pub = self.create_publisher(String, '/referee/robot_dynamic', qos_reliable)
        self.robot_static_pub = self.create_publisher(String, '/referee/robot_static', qos_reliable)
        self.robot_module_pub = self.create_publisher(String, '/referee/robot_module', qos_reliable)
        self.robot_position_pub = self.create_publisher(String, '/referee/robot_position', qos_reliable)
        self.buff_pub = self.create_publisher(String, '/referee/buff', qos_reliable)
        self.penalty_pub = self.create_publisher(String, '/referee/penalty', qos_reliable)
        self.path_plan_pub = self.create_publisher(String, '/referee/path_plan', qos_reliable)
        self.map_click_pub = self.create_publisher(String, '/referee/map_click', qos_reliable)
        self.radar_info_pub = self.create_publisher(String, '/referee/radar_info', qos_reliable)
        self.tech_core_pub = self.create_publisher(String, '/referee/tech_core', qos_reliable)
        self.perf_sync_pub = self.create_publisher(String, '/referee/perf_sync', qos_reliable)
        self.deploy_mode_pub = self.create_publisher(String, '/referee/deploy_mode', qos_reliable)
        self.rune_sync_pub = self.create_publisher(String, '/referee/rune_status', qos_reliable)
        self.sentry_status_pub = self.create_publisher(String, '/referee/sentry_status', qos_reliable)
        self.dart_target_pub = self.create_publisher(String, '/referee/dart_target', qos_reliable)
        self.sentry_ctrl_res_pub = self.create_publisher(String, '/referee/sentry_ctrl_result', qos_reliable)
        self.airsupport_pub = self.create_publisher(String, '/referee/airsupport', qos_reliable)
        
        # 上行指令订阅
        self.keyboard_sub = self.create_subscription(
            String, '/referee/cmd/keyboard', self._on_keyboard_cmd, qos_best_effort
        )
        self.custom_ctrl_sub = self.create_subscription(
            String, '/referee/cmd/custom_control', self._on_custom_cmd, qos_best_effort
        )
        self.common_cmd_sub = self.create_subscription(
            String, '/referee/cmd/common', self._on_common_cmd, qos_reliable
        )
        
        # MQTT客户端
        self.mqtt_client = mqtt.Client(
            client_id=str(self.robot_id),
            clean_session=True,
            protocol=mqtt.MQTTv311
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        self.mqtt_thread = threading.Thread(target=self._mqtt_loop, daemon=True)
        self.mqtt_thread.start()
        
        if self.enable_video:
            self.cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
            self.cleanup_thread.start()
        
        self.create_timer(2.0, self._print_stats)
        self.get_logger().info(f'Full referee bridge starting, connecting to {self.mqtt_ip}:{self.mqtt_port}')
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('✅ MQTT connected, subscribing all topics')
            subscribe_topics = [
                ('GameStatus', 1),
                ('GlobalUnitStatus', 1),
                ('GlobalLogisticsStatus', 1),
                ('GlobalSpecialMechanism', 1),
                ('Event', 1),
                ('RobotInjuryStat', 1),
                ('RobotRespawnStatus', 1),
                ('RobotStaticStatus', 1),
                ('RobotDynamicStatus', 1),
                ('RobotModuleStatus', 1),
                ('RobotPosition', 1),
                ('Buff', 1),
                ('PenaltyInfo', 1),
                ('RobotPathPlanInfo', 1),
                ('MapClickInfo', 1),
                ('RadarInfoToClient', 1),
                ('CustomByteBlock', 1),
                ('TechCoreMotionStateSync', 1),
                ('RobotPerformanceSelectionSync', 1),
                ('DeployModeStatusSync', 1),
                ('RuneStatusSync', 1),
                ('SentryStatusSync', 1),
                ('DartSelectTargetStatusSync', 1),
                ('SentryCtrlResult', 1),
                ('AirSupportStatusSync', 1),
            ]
            for topic, qos in subscribe_topics:
                client.subscribe(topic, qos=qos)
        else:
            self.get_logger().error(f'MQTT connect failed, code: {rc}')
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f'MQTT disconnected, code: {rc}')
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload
            
            if topic == 'CustomByteBlock' and self.enable_video:
                self._handle_custom_byte_block(payload)
                return
            
            handler_map = {
                'GameStatus': self._handle_game_status,
                'GlobalUnitStatus': self._handle_global_unit,
                'GlobalLogisticsStatus': self._handle_global_logistics,
                'GlobalSpecialMechanism': self._handle_global_mechanism,
                'Event': self._handle_event,
                'RobotInjuryStat': self._handle_robot_injury,
                'RobotRespawnStatus': self._handle_robot_respawn,
                'RobotStaticStatus': self._handle_robot_static,
                'RobotDynamicStatus': self._handle_robot_dynamic,
                'RobotModuleStatus': self._handle_robot_module,
                'RobotPosition': self._handle_robot_position,
                'Buff': self._handle_buff,
                'PenaltyInfo': self._handle_penalty,
                'RobotPathPlanInfo': self._handle_path_plan,
                'MapClickInfo': self._handle_map_click,
                'RadarInfoToClient': self._handle_radar_info,
                'TechCoreMotionStateSync': self._handle_tech_core,
                'RobotPerformanceSelectionSync': self._handle_perf_sync,
                'DeployModeStatusSync': self._handle_deploy_mode,
                'RuneStatusSync': self._handle_rune_sync,
                'SentryStatusSync': self._handle_sentry_status,
                'DartSelectTargetStatusSync': self._handle_dart_target,
                'SentryCtrlResult': self._handle_sentry_ctrl_res,
                'AirSupportStatusSync': self._handle_airsupport,
            }
            
            if topic in handler_map:
                handler_map[topic](payload)
            else:
                raw_topic = f"/referee/raw/{topic.lower()}"
                if raw_topic not in self.raw_pubs:
                    self.raw_pubs[raw_topic] = self.create_publisher(String, raw_topic, 10)
                raw_msg = String()
                raw_msg.data = payload.hex()
                self.raw_pubs[raw_topic].publish(raw_msg)
                
        except Exception as e:
            self.get_logger().error(f'Process MQTT message failed [{msg.topic}]: {e}')
    
    def _mqtt_loop(self):
        retry_delay = 1.0
        while rclpy.ok():
            try:
                self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port, keepalive=60)
                retry_delay = 1.0
                self.mqtt_client.loop_forever()
            except Exception as e:
                self.get_logger().error(f'MQTT connection error: {e}, retry in {retry_delay:.1f}s')
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 1.5, 10.0)
    
    # ========== 下行数据解析 ==========
    def _handle_game_status(self, payload):
        msg = GameStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'stage={msg.current_stage} cd={msg.stage_countdown_sec}s R:{msg.red_score} B:{msg.blue_score}'
        self.game_status_pub.publish(out)
    
    def _handle_global_unit(self, payload):
        msg = GlobalUnitStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'baseHp={msg.base_health} outpostHp={msg.outpost_health} enemyBaseHp={msg.enemy_base_health}'
        self.global_unit_pub.publish(out)
    
    def _handle_global_logistics(self, payload):
        msg = GlobalLogisticsStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'gold={msg.remaining_economy} techLv={msg.tech_level} encryptLv={msg.encryption_level}'
        self.global_logistics_pub.publish(out)
    
    def _handle_global_mechanism(self, payload):
        msg = GlobalSpecialMechanism()
        msg.ParseFromString(payload)
        ids = ",".join(map(str, msg.mechanism_id))
        times = ",".join(map(str, msg.mechanism_time_sec))
        out = String()
        out.data = f'mech_ids=[{ids}] mech_times=[{times}]'
        self.global_mechanism_pub.publish(out)
    
    def _handle_event(self, payload):
        msg = Event()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'event_id={msg.event_id} param={msg.param}'
        self.event_pub.publish(out)
    
    def _handle_robot_injury(self, payload):
        msg = RobotInjuryStat()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'dmg={msg.total_damage} killer={msg.killer_id} coll={msg.collision_damage}'
        self.robot_injury_pub.publish(out)
    
    def _handle_robot_respawn(self, payload):
        msg = RobotRespawnStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'pending={msg.is_pending_respawn} progress={msg.current_respawn_progress} free={msg.can_free_respawn}'
        self.robot_respawn_pub.publish(out)
    
    def _handle_robot_static(self, payload):
        msg = RobotStaticStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'rid={msg.robot_id} type={msg.robot_type} lv={msg.level} maxHp={msg.max_health}'
        self.robot_static_pub.publish(out)
    
    def _handle_robot_dynamic(self, payload):
        msg = RobotDynamicStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'hp={msg.current_health} heat={msg.current_heat:.1f} bufJ={msg.current_buffer_energy} ammo={msg.remaining_ammo}'
        self.robot_dynamic_pub.publish(out)
    
    def _handle_robot_module(self, payload):
        msg = RobotModuleStatus()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'pm={msg.power_manager} cam={msg.video_transmission} shooterS={msg.small_shooter} shooterB={msg.big_shooter}'
        self.robot_module_pub.publish(out)
    
    def _handle_robot_position(self, payload):
        msg = RobotPosition()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'x={msg.x:.2f} y={msg.y:.2f} yaw={msg.yaw:.1f} rid={msg.robot_id}'
        self.robot_position_pub.publish(out)
    
    def _handle_buff(self, payload):
        msg = Buff()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'rid={msg.robot_id} type={msg.buff_type} lv={msg.buff_level} left={msg.buff_left_time}s'
        self.buff_pub.publish(out)
    
    def _handle_penalty(self, payload):
        msg = PenaltyInfo()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'type={msg.penalty_type} dur={msg.penalty_effect_sec} total={msg.total_penalty_num}'
        self.penalty_pub.publish(out)
    
    def _handle_path_plan(self, payload):
        msg = RobotPathPlanInfo()
        msg.ParseFromString(payload)
        xs = ",".join(map(str, msg.offset_x))
        ys = ",".join(map(str, msg.offset_y))
        out = String()
        out.data = f'intention={msg.intention} startX={msg.start_pos_x} startY={msg.start_pos_y} offX=[{xs}] offY=[{ys}] sender={msg.sender_id}'
        self.path_plan_pub.publish(out)
    
    def _handle_map_click(self, payload):
        msg = MapClickInfo()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'sendAll={msg.is_send_all} mode={msg.mode} type={msg.type} x={msg.map_x:.1f} y={msg.map_y:.1f}'
        self.map_click_pub.publish(out)
    
    def _handle_radar_info(self, payload):
        msg = RadarInfoToClient()
        arr = ",".join(map(str, msg.radar_single_robot_info))
        out = String()
        out.data = f'radar_robot_list=[{arr}]'
        self.radar_info_pub.publish(out)
    
    def _handle_tech_core(self, payload):
        msg = TechCoreMotionStateSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'baseSt={msg.basic_state} moveSt={msg.move_state} enemyCore={msg.enemy_core_status} allRemain={msg.remain_time_all}'
        self.tech_core_pub.publish(out)
    
    def _handle_perf_sync(self, payload):
        msg = RobotPerformanceSelectionSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'shooter={msg.shooter} chassis={msg.chassis} sentryCtrl={msg.sentry_control}'
        self.perf_sync_pub.publish(out)
    
    def _handle_deploy_mode(self, payload):
        msg = DeployModeStatusSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'deploy_status={msg.status}'
        self.deploy_mode_pub.publish(out)
    
    def _handle_rune_sync(self, payload):
        msg = RuneStatusSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'runeSt={msg.rune_status} arms={msg.activated_arms} avgRing={msg.average_rings:.2f}'
        self.rune_sync_pub.publish(out)
    
    def _handle_sentry_status(self, payload):
        msg = SentryStatusSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'posture={msg.posture_id} weakened={msg.is_weakened}'
        self.sentry_status_pub.publish(out)
    
    def _handle_dart_target(self, payload):
        msg = DartSelectTargetStatusSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'target={msg.target_id} open={msg.open}'
        self.dart_target_pub.publish(out)
    
    def _handle_sentry_ctrl_res(self, payload):
        msg = SentryCtrlResult()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'cmdId={msg.command_id} code={msg.result_code}'
        self.sentry_ctrl_res_pub.publish(out)
    
    def _handle_airsupport(self, payload):
        msg = AirSupportStatusSync()
        msg.ParseFromString(payload)
        out = String()
        out.data = f'status={msg.airsupport_status} left={msg.left_time} cost={msg.cost_coins} locked={msg.is_being_targeted}'
        self.airsupport_pub.publish(out)
    
    # ========== 视频流处理 ==========
    def _handle_custom_byte_block(self, payload):
        block = CustomByteBlock()
        block.ParseFromString(payload)
        self._process_block_data(block.data)
    
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
    
    def _process_block_data(self, data):
        self.recv_block_count += 1
        if len(data) < 8:
            return
        
        frame_id = struct.unpack('!H', data[0:2])[0]
        chunk_idx = struct.unpack('!H', data[2:4])[0]
        total_bytes = struct.unpack('!I', data[4:8])[0]
        payload = data[8:]
        
        max_chunk = (total_bytes + 1023) // 1024
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
        
        if chunk_idx not in frame['chunks']:
            frame['chunks'][chunk_idx] = payload
            frame['received'] += len(payload)
        
        if len(frame['chunks']) == frame['max_chunk'] and frame['received'] >= frame['total']:
            sorted_keys = sorted(frame['chunks'].keys())
            complete_frame = b''.join([frame['chunks'][k] for k in sorted_keys])
            self._publish_frame(complete_frame)
            self.complete_frame_count += 1
            del self.frame_buffer[frame_id]
    
    def _publish_frame(self, frame_data):
        chunk_size = self.packet_size
        for i in range(0, len(frame_data), chunk_size):
            chunk = frame_data[i:i+chunk_size]
            msg = VideoPacket()
            msg.sequence_id = self.sequence_id
            msg.timestamp_ns = self.get_clock().now().nanoseconds
            # 核心修复：移除尾部补0
            msg.data = array('B', chunk)
            self.packet_pub.publish(msg)
            self.sequence_id += 1
    
    # ========== 上行指令 ==========
    def _on_keyboard_cmd(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) < 7:
                return
            ctrl = KeyboardMouseControl()
            ctrl.mouse_x = int(parts[0])
            ctrl.mouse_y = int(parts[1])
            ctrl.mouse_z = int(parts[2])
            ctrl.left_button_down = parts[3] == '1'
            ctrl.right_button_down = parts[4] == '1'
            ctrl.keyboard_value = int(parts[5])
            ctrl.mid_button_down = parts[6] == '1'
            self.mqtt_client.publish('KeyboardMouseControl', ctrl.SerializeToString(), qos=1)
        except Exception as e:
            self.get_logger().error(f'Publish keyboard cmd failed: {e}')
    
    def _on_custom_cmd(self, msg):
        try:
            ctrl = CustomControl()
            ctrl.data = msg.data.encode()[:30]
            self.mqtt_client.publish('CustomControl', ctrl.SerializeToString(), qos=1)
        except Exception as e:
            self.get_logger().error(f'Publish custom cmd failed: {e}')
    
    def _on_common_cmd(self, msg):
        try:
            parts = msg.data.split(',')
            if len(parts) < 2:
                return
            cmd = CommonCommand()
            cmd.cmd_type = int(parts[0])
            cmd.param = int(parts[1])
            self.mqtt_client.publish('CommonCommand', cmd.SerializeToString(), qos=1)
        except Exception as e:
            self.get_logger().error(f'Publish common cmd failed: {e}')
    
    # ========== 辅助函数 ==========
    def _print_stats(self):
        self.get_logger().info(
            f'Stats: {self.recv_block_count} blocks, {self.complete_frame_count} frames'
        )
    
    def _cleanup_loop(self):
        while rclpy.ok():
            now = time.time()
            expired = [fid for fid, f in self.frame_buffer.items() 
                       if now - f['ts'] > self.frame_timeout_s]
            for fid in expired:
                self.lost_frame_count += 1
                del self.frame_buffer[fid]
            time.sleep(0.5)
    
    def destroy_node(self):
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FullRefereeBridge()
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
