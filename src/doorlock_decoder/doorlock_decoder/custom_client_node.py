#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import paho.mqtt.client as mqtt
import threading
from std_msgs.msg import String, ByteMultiArray
from doorlock_sniper.msg import VideoPacketVar
from array import array
from google.protobuf.json_format import MessageToJson
# 同包导入pb文件
from doorlock_decoder import custom_protocol_pb2 as pb

class CustomClientNode(Node):
    def __init__(self):
        super().__init__('custom_client_node')
        # 开启参数动态类型转换，允许数字自动转字符串
        str_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('mqtt_broker_ip', '192.168.12.1', str_desc)
        self.declare_parameter('mqtt_broker_port', 3333)
        self.declare_parameter('client_id', '101', str_desc)

        # 获取参数，强制转为字符串
        ip = self.get_parameter('mqtt_broker_ip').value
        port = self.get_parameter('mqtt_broker_port').value
        client_id_raw = self.get_parameter('client_id').value
        self.client_id = str(client_id_raw)

        # 发布器
        self.general_pub = self.create_publisher(String, '/custom_protocol', 10)
        self.custom_video_pub = self.create_publisher(VideoPacketVar, '/custom_video_stream_packet', 100)
        self.packet_seq_id = 0
        # MQTT 客户端
        self.mqtt_client = mqtt.Client(client_id=self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(ip, port, 60)
        self.mqtt_thread = threading.Thread(target=self.mqtt_loop, daemon=True)
        self.mqtt_thread.start()
        self.get_logger().info(f'CustomClientNode started, broker={ip}:{port}, client_id={self.client_id}')
    def mqtt_loop(self):
        self.mqtt_client.loop_forever()
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            topics = [
                'GameStatus', 'GlobalUnitStatus', 'GlobalLogisticsStatus',
                'GlobalSpecialMechanism', 'Event', 'RobotInjuryStat',
                'RobotRespawnStatus', 'RobotStaticStatus', 'RobotDynamicStatus',
                'RobotModuleStatus', 'RobotPosition', 'Buff', 'PenaltyInfo',
                'RobotPathPlanInfo', 'MapClickInfo', 'RadarInfoToClient',
                'CustomByteBlock', 'TechCoreMotionStateSync',
                'RobotPerformanceSelectionSync', 'DeployModeStatusSync',
                'RuneStatusSync', 'SentryStatusSync', 'DartSelectTargetStatusSync',
                'SentryCtrlResult', 'AirSupportStatusSync'
            ]
            for topic in topics:
                client.subscribe(topic, qos=1)
                self.get_logger().info(f'Subscribed to {topic}')
        else:
            self.get_logger().error(f'MQTT connect failed, rc={rc}')
    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload
        try:
            if topic == 'GameStatus':
                pb_msg = pb.GameStatus()
            elif topic == 'GlobalUnitStatus':
                pb_msg = pb.GlobalUnitStatus()
            elif topic == 'GlobalLogisticsStatus':
                pb_msg = pb.GlobalLogisticsStatus()
            elif topic == 'GlobalSpecialMechanism':
                pb_msg = pb.GlobalSpecialMechanism()
            elif topic == 'Event':
                pb_msg = pb.Event()
            elif topic == 'RobotInjuryStat':
                pb_msg = pb.RobotInjuryStat()
            elif topic == 'RobotRespawnStatus':
                pb_msg = pb.RobotRespawnStatus()
            elif topic == 'RobotStaticStatus':
                pb_msg = pb.RobotStaticStatus()
            elif topic == 'RobotDynamicStatus':
                pb_msg = pb.RobotDynamicStatus()
            elif topic == 'RobotModuleStatus':
                pb_msg = pb.RobotModuleStatus()
            elif topic == 'RobotPosition':
                pb_msg = pb.RobotPosition()
            elif topic == 'Buff':
                pb_msg = pb.Buff()
            elif topic == 'PenaltyInfo':
                pb_msg = pb.PenaltyInfo()
            elif topic == 'RobotPathPlanInfo':
                pb_msg = pb.RobotPathPlanInfo()
            elif topic == 'MapClickInfo':
                pb_msg = pb.MapClickInfo()
            elif topic == 'RadarInfoToClient':
                pb_msg = pb.RadarInfoToClient()
            elif topic == 'CustomByteBlock':
                pb_msg = pb.CustomByteBlock()
            elif topic == 'TechCoreMotionStateSync':
                pb_msg = pb.TechCoreMotionStateSync()
            elif topic == 'RobotPerformanceSelectionSync':
                pb_msg = pb.RobotPerformanceSelectionSync()
            elif topic == 'DeployModeStatusSync':
                pb_msg = pb.DeployModeStatusSync()
            elif topic == 'RuneStatusSync':
                pb_msg = pb.RuneStatusSync()
            elif topic == 'SentryStatusSync':
                pb_msg = pb.SentryStatusSync()
            elif topic == 'DartSelectTargetStatusSync':
                pb_msg = pb.DartSelectTargetStatusSync()
            elif topic == 'SentryCtrlResult':
                pb_msg = pb.SentryCtrlResult()
            elif topic == 'AirSupportStatusSync':
                pb_msg = pb.AirSupportStatusSync()
            else:
                self.get_logger().warn(f'Unhandled topic: {topic}')
                return
            pb_msg.ParseFromString(payload)
            if topic == 'CustomByteBlock':
                packet = VideoPacketVar()
                packet.sequence_id = self.packet_seq_id
                packet.timestamp_ns = self.get_clock().now().nanoseconds
                packet.data = array('B', pb_msg.data)
                self.custom_video_pub.publish(packet)
                self.packet_seq_id += 1
            else:
                json_str = MessageToJson(pb_msg)
                ros_msg = String()
                ros_msg.data = json_str
                self.general_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing {topic}: {e}')
    def destroy_node(self):
        self.mqtt_client.disconnect()
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = CustomClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
