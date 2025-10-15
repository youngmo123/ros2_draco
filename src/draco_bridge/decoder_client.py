# src/draco_bridge/decoder_client.py
import socket
import struct
import threading
import time
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from draco_bridge.utils import decode_draco

def get_local_ip():
    """로컬 IP 주소 자동 감지"""
    try:
        # 외부 서버(Google DNS)에 연결 시도하여 로컬 IP 확인
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        # 실패 시 localhost 반환
        return '127.0.0.1'

def discover_draco_server(port=50051, timeout=2.0, logger=None):
    """Draco 서버 자동 탐지"""
    try:
        # 로컬 네트워크 대역 스캔
        local_ip = get_local_ip()
        base_ip = '.'.join(local_ip.split('.')[:-1])  # 192.168.0.x에서 192.168.0 추출
        
        if logger:
            logger.info(f'[Decoder] Searching for Draco server in {base_ip}.x range...')
        
        for i in range(1, 255):  # 1-254 범위 스캔
            target_ip = f'{base_ip}.{i}'
            if target_ip == local_ip:
                continue  # 자기 자신은 건너뛰기
                
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                result = sock.connect_ex((target_ip, port))
                sock.close()
                
                if result == 0:  # 연결 성공
                    if logger:
                        logger.info(f'[Decoder] Found Draco server at {target_ip}:{port}')
                    return target_ip
                    
            except Exception:
                continue
                
        if logger:
            logger.warn(f'[Decoder] No Draco server found in network range')
        return None
        
    except Exception as e:
        if logger:
            logger.error(f'[Decoder] Server discovery failed: {e}')
        return None

class DecoderClient(Node):
    def __init__(self):
        super().__init__('draco_decoder_client')

        # 파라미터 선언
        self.declare_parameter('tcp_server_ip', 'auto')
        self.declare_parameter('tcp_server_port', 50051)
        self.declare_parameter('output_topic', '/sensing/lidar/points_raw')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_depth', 10)
        
        # 파라미터 값 가져오기
        default_ip = self.get_parameter('tcp_server_ip').get_parameter_value().string_value
        if default_ip == 'auto':
            # 먼저 서버 자동 탐지 시도
            self.tcp_server_ip = discover_draco_server(
                port=self.get_parameter('tcp_server_port').get_parameter_value().integer_value,
                timeout=1.0,
                logger=self.get_logger()
            )
            if self.tcp_server_ip is None:
                # 서버 탐지 실패 시 로컬 IP 사용 (같은 컴퓨터에서 테스트)
                self.tcp_server_ip = get_local_ip()
                self.get_logger().info(f'[Decoder] Auto-detected IP: {self.tcp_server_ip}')
        else:
            self.tcp_server_ip = default_ip
        
        self.tcp_server_port = self.get_parameter('tcp_server_port').get_parameter_value().integer_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.qos_reliability = self.get_parameter('qos_reliability').get_parameter_value().string_value
        self.qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        # QoS 설정
        if self.qos_reliability == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=self.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,  # RELIABLE로 변경
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)

        self.sock = None
        self.connected = False
        self.running = True
        self.template_msg = None  # 첫 번째 메시지를 템플릿으로 사용
        
        # 모니터 URL 설정 - 자동으로 송신 PC IP 감지
        self.monitor_urls = self._setup_monitor_urls()
        self.monitor_url = self.monitor_urls[0]  # 기본값

        threading.Thread(target=self.connect_and_receive_loop, daemon=True).start()
        threading.Thread(target=self.status_report_loop, daemon=True).start()

    def _setup_monitor_urls(self):
        """모니터 URL 설정 - 자동으로 송신 PC IP 감지"""
        monitor_urls = []
        
        # 1. 송신 PC IP를 자동 감지하여 추가
        if self.tcp_server_ip:
            server_ip = self.tcp_server_ip
            monitor_urls.append(f"http://{server_ip}:5000/api/decoder_status")
            self.get_logger().info(f'[Monitor] Auto-detected server IP for monitor: {server_ip}')
        
        # 2. 로컬호스트 대안들 추가
        monitor_urls.extend([
            "http://localhost:5000/api/decoder_status",
            "http://127.0.0.1:5000/api/decoder_status"
        ])
        
        self.get_logger().info(f'[Monitor] Monitor URLs configured: {monitor_urls}')
        return monitor_urls

    def connect_and_receive_loop(self):
        while self.running:
            try:
                if not self.connected:
                    self.connect_to_server()
                
                if self.connected:
                    self.receive_data()
                    
            except Exception as e:
                self.get_logger().error(f'Connection error: {e}')
                self.connected = False
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None
                
                if self.running:
                    self.get_logger().info(f'Reconnecting in 3 seconds...')
                    time.sleep(3)

    def connect_to_server(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)  # Keep-alive 활성화
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)  # 1초 후 keep-alive 시작
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)  # 3초 간격
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)    # 5번 시도
            self.sock.settimeout(15.0)  # 타임아웃 증가
            self.sock.connect((self.tcp_server_ip, self.tcp_server_port))
            self.connected = True
            self.get_logger().info(f'[Decoder] Connected to server {self.tcp_server_ip}:{self.tcp_server_port}')
            
            # 연결 성공 시 모니터에 상태 보고
            self.send_status_to_monitor("connected")
            
        except Exception as e:
            self.get_logger().error(f'[Decoder] Failed to connect to server: {e}')
            self.connected = False
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
            
            # 연결 실패 시 모니터에 상태 보고
            self.send_status_to_monitor("disconnected")

    def receive_data(self):
        try:
            while self.connected and self.running:
                # 데이터 크기 헤더 수신 (4바이트)
                size_header = self._receive_exact_bytes(4)
                if not size_header:
                    break
                
                data_size = struct.unpack('<I', size_header)[0]
                
                # 실제 데이터 수신
                data = self._receive_exact_bytes(data_size)
                if not data:
                    break
                
                # 데이터 처리
                self.process_received_data(data)
                
        except Exception as e:
            self.get_logger().error(f'[Decoder] Error receiving data: {e}')
            self.connected = False

    def _receive_exact_bytes(self, num_bytes):
        """정확한 바이트 수만큼 수신"""
        data = b''
        while len(data) < num_bytes:
            try:
                chunk = self.sock.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'[Decoder] Error receiving bytes: {e}')
                return None
        return data

    def process_received_data(self, data):
        """수신된 데이터 처리 및 포인트 클라우드 발행"""
        try:
            self.get_logger().info(f'[Decoder] Processing received data: {len(data)} bytes')
            
            # 템플릿 메시지가 없으면 기본 템플릿 생성
            if self.template_msg is None:
                from sensor_msgs.msg import PointField
                self.template_msg = PointCloud2()
                self.template_msg.header.frame_id = self.frame_id
                self.template_msg.is_bigendian = False
                self.template_msg.is_dense = True  # True로 설정하여 RViz2가 포인트를 올바르게 렌더링
                # 기본 필드 설정
                self.template_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                self.get_logger().info(f'[Decoder] Created template message with frame_id: {self.frame_id}')
            
            # Draco 압축 해제
            self.get_logger().info(f'[Decoder] Starting Draco decompression...')
            pointcloud_msg = decode_draco(data, self.template_msg)
            
            # 완전히 새로운 메시지 생성
            new_msg = PointCloud2()
            # 현재 시간으로 타임스탬프 설정
            new_msg.header.stamp = self.get_clock().now().to_msg()
            # frame_id는 고정 (변경하면 안됨!)
            new_msg.header.frame_id = self.frame_id
            new_msg.height = pointcloud_msg.height
            new_msg.width = pointcloud_msg.width
            new_msg.fields = pointcloud_msg.fields
            new_msg.is_bigendian = pointcloud_msg.is_bigendian
            new_msg.point_step = pointcloud_msg.point_step
            new_msg.row_step = pointcloud_msg.row_step
            new_msg.is_dense = pointcloud_msg.is_dense
            new_msg.data = pointcloud_msg.data  # 데이터 복사
            
            # 포인트 클라우드 크기 검증 및 조정
            if new_msg.width < 10:
                self.get_logger().warn(f'[Decoder] Pointcloud too small: {new_msg.width}x{new_msg.height}, skipping...')
                return
            
            # 토픽 발행
            self.get_logger().info(f'[Decoder] Publishing NEW pointcloud: {new_msg.width}x{new_msg.height}, {len(new_msg.data)} bytes, stamp: {new_msg.header.stamp}')
            self.pub.publish(new_msg)
            self.get_logger().info(f'[Decoder] Successfully published NEW pointcloud!')
            
        except Exception as e:
            self.get_logger().error(f'[Decoder] Error processing received data: {e}')
            import traceback
            self.get_logger().error(f'[Decoder] Traceback: {traceback.format_exc()}')

    def status_report_loop(self):
        """주기적으로 상태를 Flask 모니터에 보고"""
        last_reported_status = None
        report_count = 0
        consecutive_failures = 0
        
        while self.running:
            try:
                current_status = "connected" if self.connected else "disconnected"
                
                # 상태가 변경되었거나 30초마다 보고 (더 안정적으로)
                should_report = (current_status != last_reported_status or report_count % 6 == 0)
                
                if should_report:
                    success = self.send_status_to_monitor(current_status)
                    if success:
                        last_reported_status = current_status
                        consecutive_failures = 0
                        self.get_logger().info(f'Status reported: {current_status}')
                    else:
                        consecutive_failures += 1
                        self.get_logger().warn(f'Failed to report status: {current_status} (failures: {consecutive_failures})')
                        
                        # 연속 실패가 3회 이상이면 잠시 대기
                        if consecutive_failures >= 3:
                            self.get_logger().warn(f'Too many failures, waiting 30 seconds...')
                            time.sleep(30)
                            consecutive_failures = 0
                
                report_count += 1
                time.sleep(5)  # 5초마다 상태 확인
            except Exception as e:
                self.get_logger().warn(f'Status report error: {e}')
                time.sleep(10)

    def send_status_to_monitor(self, status):
        """디코더 상태를 Flask 모니터에 전송"""
        data = {
            'decoder_status': status,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        # 여러 URL 시도
        for url in self.monitor_urls:
            try:
                response = requests.post(url, json=data, timeout=2)
                if response.status_code == 200:
                    self.get_logger().debug(f'Status sent to monitor ({url}): {status}')
                    return True
            except Exception as e:
                self.get_logger().debug(f'Monitor report failed ({url}): {e}')
                continue
        
        self.get_logger().warn(f'All monitor URLs failed for status: {status}')
        return False

    def destroy_node(self):
        """노드 종료 시 정리"""
        self.running = False
        self.connected = False
        
        # 종료 시 모니터에 상태 보고
        self.send_status_to_monitor("disconnected")
        
        if self.sock:
            self.sock.close()
        
        super().destroy_node()

def main():
    rclpy.init()
    node = DecoderClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()