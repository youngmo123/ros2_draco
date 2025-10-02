# src/draco_bridge/encoder_server.py
import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ByteMultiArray
from draco_bridge.utils import encode_draco

class EncoderServer(Node):
    def __init__(self):
        super().__init__('draco_encoder_server')

        self.input_topic = self.declare_parameter('input_topic', '/sensing/lidar/top/pointcloud').get_string_value()
        self.output_topic = self.declare_parameter('output_topic', '/lidar_compressed').get_string_value()
        self.tcp_bind_ip = self.declare_parameter('tcp_bind_ip', '127.0.0.1').get_string_value()
        self.tcp_port = self.declare_parameter('tcp_port', 50051).get_parameter_value().integer_value

        # QoS: 센서 스트림 구독은 보통 Best Effort
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb_cloud, sub_qos)

        # 디버그/모니터용 압축 바이트 토픽
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub_compressed = self.create_publisher(ByteMultiArray, self.output_topic, pub_qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.tcp_bind_ip, self.tcp_port))
        self.sock.listen(1)
        self.get_logger().info(f'[Encoder] TCP Server listening on {self.tcp_bind_ip}:{self.tcp_port}')

        self.client_conn = None
        threading.Thread(target=self.accept_thread, daemon=True).start()

    def accept_thread(self):
        while True:
            conn, addr = self.sock.accept()
            self.get_logger().info(f'[Encoder] Client connected: {addr}')
            self.client_conn = conn

    def cb_cloud(self, msg: PointCloud2):
        # Draco 압축 → ByteMultiArray 발행 + TCP 전송
        comp = encode_draco(msg)

        bma = ByteMultiArray()
        bma.data = list(comp)
        self.pub_compressed.publish(bma)

        # TCP: 길이 프리픽스 + payload
        if self.client_conn:
            try:
                length = len(comp).to_bytes(4, 'little')
                self.client_conn.sendall(length + comp)
            except Exception as e:
                self.get_logger().warn(f'[Encoder] send failed: {e}')
                self.client_conn = None

def main():
    rclpy.init()
    node = EncoderServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
