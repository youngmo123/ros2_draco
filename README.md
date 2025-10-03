# ROS2 Draco Point Cloud Compression Bridge

ROS2 패키지로 Google Draco 3D 압축 라이브러리를 사용하여 PointCloud2 메시지를 압축/해제하고 TCP를 통해 전송합니다.

## 특징

- ✅ Google Draco 3D 압축 (실패 시 zlib fallback)
- ✅ 약 60% 압축률 달성
- ✅ TCP 소켓 통신으로 네트워크 전송
- ✅ 자동 IP 주소 감지
- ✅ ROS2 launch 파일 제공

## 의존성

- ROS2 (Humble/Foxy)
- Python 3
- sensor_msgs
- numpy
- Draco 라이브러리 (~/draco/build/)

## 빌드

```bash
cd ~/ros2_draco
colcon build --packages-select draco_bridge
```

## 사용법

### 1. Encoder Server (데이터 송신 측)

rosbag을 재생하면서 압축하여 전송:

```bash
# 새로운 터미널
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge encoder_sender.launch.py
```

**파라미터:**
- `bag`: rosbag 파일 경로 (기본값: `/home/youngmo/Downloads/rosbag2_2024_09_24-14_28_57`)
- `input_topic`: 입력 포인트 클라우드 토픽 (기본값: `/sensing/lidar/top/pointcloud`)
- `bind_ip`: 바인딩 IP (기본값: `auto` - 자동 감지)
- `port`: TCP 포트 (기본값: `50051`)

**예시:**
```bash
ros2 launch draco_bridge encoder_sender.launch.py \
    bag:=/path/to/your/bag \
    bind_ip:=192.168.1.100 \
    port:=50051
```

### 2. Decoder Client (데이터 수신 측)

압축된 데이터를 수신하여 해제 후 발행:

```bash
# 새로운 터미널
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge decoder_receiver.launch.py
```

**파라미터:**
- `host`: 서버 IP (기본값: `auto` - 자동 감지)
- `port`: TCP 포트 (기본값: `50051`)
- `output_topic`: 출력 토픽 (기본값: `/sensing/lidar/points_raw`)
- `frame_id`: 프레임 ID (기본값: `lidar_link`)
- `reliability`: QoS 신뢰성 (`reliable` 또는 `best_effort`)

**예시:**
```bash
ros2 launch draco_bridge decoder_receiver.launch.py \
    host:=192.168.1.100 \
    port:=50051 \
    reliability:=best_effort
```

## 자동 IP 감지

기본값으로 `auto`를 사용하면 시스템이 자동으로 네트워크 IP를 감지합니다:

```bash
# 자동 IP 감지 (권장)
ros2 launch draco_bridge encoder_sender.launch.py

# 수동 IP 지정
ros2 launch draco_bridge encoder_sender.launch.py bind_ip:=192.168.1.100
```

## 토픽

### Encoder Server
- **입력**: `/sensing/lidar/top/pointcloud` (PointCloud2)
- **출력**: `/lidar_compressed` (ByteMultiArray) - 디버그용

### Decoder Client
- **출력**: `/sensing/lidar/points_raw` (PointCloud2)

## 압축 성능

- **압축률**: 약 60% (원본 대비 40% 크기)
- **압축 방식**: Google Draco 3D 압축
- **Fallback**: Draco 실패 시 zlib 압축 자동 전환

## 문제 해결

### Launch 파일을 찾을 수 없는 경우

```bash
cd ~/ros2_draco
rm -rf build install log
colcon build --packages-select draco_bridge
source install/setup.bash
```

### 환경 변수 오류

새로운 터미널을 열고 다시 시도하세요:

```bash
cd ~/ros2_draco
source /opt/ros/humble/setup.bash  # 또는 foxy
source install/setup.bash
```

## 라이센스

Apache-2.0

## 유지관리자

youngmo (you@example.com)

