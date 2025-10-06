# ROS2 Draco Point Cloud Compression Bridge

ROS2 패키지로 Google Draco 3D 압축 라이브러리를 사용하여 PointCloud2 메시지를 압축/해제하고 TCP를 통해 전송합니다.

## 특징

- ✅ Google Draco 3D 압축 
- ✅ 약 90% 압축률 달성
- ✅ TCP 소켓 통신으로 네트워크 전송
- ✅ 자동 IP 주소 감지 및 네트워크 환경 적응
- ✅ ROS2 launch 파일 제공
- ✅ 실시간 웹 모니터링 시스템 (Flask)
- ✅ 전문적인 네트워크 대역폭 측정 (iPerf 스타일)
- ✅ 다중 rosbag 자동 재생 기능

## 의존성

- ROS2 (Humble/Foxy)
- Python 3
- sensor_msgs
- numpy
- Draco 라이브러리 (~/draco/build/)
- Flask
- Flask-SocketIO
- psutil
- requests

## 빌드

```bash
cd ~/ros2_draco
colcon build --packages-select draco_bridge
```

## 사용법

### 🚀 빠른 시작 (3단계)

#### 1️⃣ 송신 PC (데이터 전송)
```bash
# 터미널 1: Encoder Server + rosbag 재생
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge encoder_sender.launch.py
```

#### 2️⃣ 수신 PC (데이터 수신)
```bash
# 터미널 1: Decoder Client
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge decoder_receiver.launch.py
```

#### 3️⃣ 모니터링 (송신 PC)
```bash
# 터미널 2: 웹 모니터링 서버
cd ~/ros2_draco
python3 src/draco_bridge/flask_monitor.py
```

웹 브라우저에서 `http://localhost:5000` 접속하여 실시간 모니터링 확인

---

### 📋 상세 사용법

#### 1. Encoder Server (데이터 송신 측)

**기본 실행:**
```bash
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge encoder_sender.launch.py
```

**특징:**
- `~/ros2bagfile` 디렉토리의 모든 rosbag 폴더를 자동으로 순차 재생
- 자동 IP 감지로 네트워크 환경에 적응
- 실시간 압축률 및 통계를 Flask 모니터에 전송

**파라미터:**
- `bag`: rosbag 파일 경로 (기본값: 자동 감지)
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
**모든 실행이 끝나고 프로세스 종료**

lsof -i :50051

kill -9 [PID]

lsof -i :50051

#### 2. Decoder Client (데이터 수신 측)

**기본 실행:**
```bash
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge decoder_receiver.launch.py
```

**특징:**
- 자동으로 송신 PC에 연결 시도
- 연결 상태를 Flask 모니터에 주기적으로 보고
- TCP Keep-Alive로 안정적인 연결 유지
- 압축된 데이터를 실시간으로 해제하여 PointCloud2 발행

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

**모든 실행이 끝나고 프로세스 종료**

lsof -i :50051

kill -9 [PID]

lsof -i :50051


#### 3. 웹 모니터링 시스템

**실시간 모니터링:**
```bash
cd ~/ros2_draco
python3 src/draco_bridge/flask_monitor.py
```

**웹 인터페이스**: `http://localhost:5000`

**모니터링 기능:**
- 🔗 **연결 상태**: 인코더 서버/디코더 클라이언트 실시간 상태
- 📊 **압축 통계**: 압축률, 원본/압축 크기, 압축률 추이 그래프
- 🌐 **네트워크 대역폭**: 실시간 업로드/다운로드 속도
- 📈 **전송 속도**: bps 단위 정확한 네트워크 성능 측정
- ⚡ **시스템 정보**: CPU, 메모리, 디스크 사용률

**네트워크 대역폭 측정:**
- iPerf 스타일 TCP 기반 대역폭 테스트
- 30초마다 자동 대역폭 측정
- 실시간 네트워크 인터페이스 모니터링
- 자동 네트워크 환경 감지 및 적응

## 🌍 네트워크 환경 적응

### 자동 IP 감지

시스템이 자동으로 네트워크 환경을 감지하고 적응합니다:

```bash
# 자동 IP 감지 (권장)
ros2 launch draco_bridge encoder_sender.launch.py

# 수동 IP 지정
ros2 launch draco_bridge encoder_sender.launch.py bind_ip:=192.168.1.100
```

### 네트워크 환경별 설정

#### 🏠 가정용 네트워크 (192.168.1.x)
```bash
# 자동 감지: 192.168.1.1 (라우터)
# 설정 저장: config/network_config.json
```

#### 🏢 사무실 네트워크 (10.0.0.x)
```bash
# 자동 감지: 10.0.0.1 (게이트웨이)
# 설정 저장: config/network_config.json
```

#### 📱 핫스팟 네트워크 (172.20.10.x)
```bash
# 자동 감지: 172.20.10.1 (핫스팟)
# 설정 저장: config/network_config.json
```

### 설정 파일 관리

**자동 설정 저장:**
```json
{
  "target_ip": "192.168.0.1",
  "last_updated": "2025-10-06 18:35:00",
  "description": "Bandwidth monitor target IP configuration"
}
```

**수동 설정:**
```bash
# 명령행으로 IP 지정
python3 src/draco_bridge/bandwidth_monitor.py 192.168.1.100

# 설정 파일 수정
vim config/network_config.json
```

## 📡 토픽 및 API

### ROS2 토픽

#### Encoder Server
- **입력**: `/sensing/lidar/top/pointcloud` (PointCloud2)
- **출력**: `/lidar_compressed` (ByteMultiArray) - 디버그용

#### Decoder Client
- **출력**: `/sensing/lidar/points_raw` (PointCloud2)

### Flask API 엔드포인트

#### 압축 통계 API
```bash
POST /api/compression_stats
Content-Type: application/json

{
  "original_size": 1024000,
  "compressed_size": 409600
}
```

#### 디코더 상태 API
```bash
POST /api/decoder_status
Content-Type: application/json

{
  "decoder_status": "connected",
  "timestamp": "2025-10-06 19:35:00"
}
```

#### 실시간 WebSocket
```javascript
// 웹 클라이언트에서 실시간 업데이트 수신
socket.on('status_update', function(data) {
    console.log('Status update:', data);
});
```

## 📊 성능 및 모니터링

### 압축 성능
- **압축률**: 약 90%
- **압축 방식**: Google Draco 3D 압축
- **실시간 모니터링**: 압축률 추이 그래프 및 통계

### 네트워크 성능
- **대역폭 측정**: iPerf 스타일 TCP 기반 테스트
- **실시간 모니터링**: 업로드/다운로드 속도 (bps 단위)
- **자동 적응**: 네트워크 환경에 따른 자동 IP 감지
- **연결 안정성**: TCP Keep-Alive 및 자동 재연결

### 시스템 모니터링
- **CPU 사용률**: 실시간 시스템 리소스 모니터링
- **메모리 사용률**: RAM 사용량 추적
- **디스크 사용률**: 저장 공간 모니터링
- **네트워크 인터페이스**: 실시간 트래픽 분석

## 🔧 문제 해결

### 일반적인 문제

#### Launch 파일을 찾을 수 없는 경우
```bash
cd ~/ros2_draco
rm -rf build install log
colcon build --packages-select draco_bridge
source install/setup.bash
```

#### 환경 변수 오류
```bash
cd ~/ros2_draco
source /opt/ros/humble/setup.bash  # 또는 foxy
source install/setup.bash
```

#### 포트 이미 사용 중 오류
```bash
# 기존 프로세스 종료
pkill -f flask_monitor
pkill -f encoder_server
pkill -f decoder_client

# 또는 특정 포트 사용 프로세스 확인
sudo netstat -tlnp | grep :5000
sudo kill -9 <PID>
```

### 네트워크 연결 문제

#### 디코더 클라이언트 연결 실패
```bash
# 1. 송신 PC IP 확인
ip addr show

# 2. 방화벽 확인
sudo ufw status

# 3. 포트 50051 열기 (필요시)
sudo ufw allow 50051

# 4. 수동 IP 지정
ros2 launch draco_bridge decoder_receiver.launch.py host:=192.168.0.16
```

#### Flask 모니터 접속 불가
```bash
# 1. Flask 모니터 상태 확인
ps aux | grep flask_monitor

# 2. 포트 5000 확인
sudo netstat -tlnp | grep :5000

# 3. 방화벽 확인
sudo ufw status
sudo ufw allow 5000  # 필요시
```

### 성능 최적화

#### 대역폭 측정 오류
```bash
# 1. 네트워크 설정 확인
python3 src/draco_bridge/bandwidth_monitor.py

# 2. 설정 파일 수정
vim config/network_config.json

# 3. 수동 IP 지정
python3 src/draco_bridge/bandwidth_monitor.py 192.168.1.100
```

#### 압축률 낮음
- rosbag 파일 크기 확인
- Draco 라이브러리 버전 확인
- 네트워크 대역폭 확인

## 📁 프로젝트 구조

```
ros2_draco/
├── src/draco_bridge/
│   ├── encoder_server.py          # 압축 서버 (송신)
│   ├── decoder_client.py          # 압축 클라이언트 (수신)
│   ├── flask_monitor.py           # 웹 모니터링 서버
│   ├── bandwidth_monitor.py       # 네트워크 대역폭 측정
│   ├── draco_wrapper.py           # Draco 압축/해제 래퍼
│   └── templates/
│       └── monitor.html           # 웹 모니터링 UI
├── launch/
│   ├── encoder_sender.launch.py   # 인코더 런치 파일
│   └── decoder_receiver.launch.py # 디코더 런치 파일
├── scripts/
│   └── play_all_bags.py           # 다중 rosbag 재생 스크립트
├── config/
│   └── network_config.json        # 네트워크 설정 파일
└── README.md                      # 프로젝트 문서
```

## 🚀 실행 순서 요약

### 1. 송신 PC (데이터 전송)
```bash
# 터미널 1: 인코더 + rosbag 재생
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge encoder_sender.launch.py

# 터미널 2: 웹 모니터링
python3 src/draco_bridge/flask_monitor.py
```

### 2. 수신 PC (데이터 수신)
```bash
# 터미널 1: 디코더 클라이언트
cd ~/ros2_draco
source install/setup.bash
ros2 launch draco_bridge decoder_receiver.launch.py
```

### 3. 모니터링 확인
- 웹 브라우저: `http://localhost:5000`
- rviz2에서 `/sensing/lidar/points_raw` 토픽 확인

## 📋 체크리스트

### 설치 확인
- [ ] ROS2 Humble 설치
- [ ] Python 3 및 필수 패키지 설치
- [ ] Draco 라이브러리 빌드
- [ ] 프로젝트 빌드 (`colcon build`)

### 실행 확인
- [ ] 송신 PC에서 인코더 실행
- [ ] 수신 PC에서 디코더 실행
- [ ] 웹 모니터링 접속
- [ ] rviz2에서 포인트 클라우드 확인

### 네트워크 확인
- [ ] 자동 IP 감지 작동
- [ ] 연결 상태 초록색 표시
- [ ] 압축률 통계 업데이트
- [ ] 대역폭 측정 정상 작동

## 라이센스

Apache-2.0

## 유지관리자

youngmo (youngmo123@hanmail.net)

