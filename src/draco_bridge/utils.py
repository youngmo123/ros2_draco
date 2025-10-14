# src/draco_bridge/utils.py
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from .draco_wrapper import encode_pointcloud_with_draco, decode_pointcloud_with_draco

def encode_draco(pc2: PointCloud2) -> bytes:
    """
    PointCloud2를 Draco로 압축 (품질 문제 시 원본 데이터 사용)
    """
    print(f"[DEBUG] encode_draco called with {len(pc2.data)} bytes")
    
    # PointCloud2 데이터를 numpy 배열로 변환
    points = pointcloud2_to_numpy(pc2)
    print(f"[DEBUG] Converted to numpy array: {points.shape}")
    print(f"[DEBUG] Original data sample (first 3 points): {points[:3].tolist()}")
    print(f"[DEBUG] Original data stats - min: {points.min(axis=0)}, max: {points.max(axis=0)}")
    
    try:
        # Draco로 압축
        compressed_data = encode_pointcloud_with_draco(points)
        
        if compressed_data is None:
            print(f"[DEBUG] Draco compression failed, using raw data")
            # Draco 압축 실패 시 원본 데이터 사용
            header = struct.pack('<III', pc2.width, pc2.height, pc2.point_step)
            return header + pc2.data
        
        print(f"[DEBUG] Draco compression successful: {len(compressed_data)} bytes")
        # Draco 압축 헤더와 함께 반환
        header = struct.pack('<III', pc2.width, pc2.height, pc2.point_step)
        return header + compressed_data
        
    except Exception as e:
        print(f"[DEBUG] Draco encoding error: {e}, using raw data")
        # 에러 발생 시 원본 데이터 사용
        header = struct.pack('<III', pc2.width, pc2.height, pc2.point_step)
        return header + pc2.data

def decode_draco(buf: bytes, template: PointCloud2) -> PointCloud2:
    """
    Draco 압축된 데이터를 PointCloud2로 해제 (Draco 전용)
    """
    print(f"[DEBUG] decode_draco called with {len(buf)} bytes")
    
    # 헤더 분리
    header_size = 12
    if len(buf) < header_size:
        print(f"[DEBUG] Buffer too small: {len(buf)} < {header_size}")
        return template
        
    width, height, point_step = struct.unpack('<III', buf[:header_size])
    compressed_data = buf[header_size:]
    
    print(f"[DEBUG] Header: width={width}, height={height}, point_step={point_step}")
    print(f"[DEBUG] Compressed data size: {len(compressed_data)}")
    print(f"[DEBUG] Expected points from header: {width * height}")
    
    try:
        # Draco로 해제 시도
        points = decode_pointcloud_with_draco(compressed_data)
        
        if points is None or len(points) == 0:
            print(f"[DEBUG] Draco decompression failed, using raw data")
            # Draco 해제 실패 시 원본 데이터 사용
            msg = PointCloud2()
            msg.header.frame_id = template.header.frame_id
            msg.header.stamp = template.header.stamp
            msg.height = height
            msg.width = width
            msg.is_bigendian = template.is_bigendian
            msg.point_step = point_step
            msg.row_step = point_step * width
            msg.is_dense = template.is_dense
            msg.fields = template.fields
            msg.data = compressed_data  # 원본 데이터 직접 사용
            print(f"[DEBUG] Using raw data: {len(msg.data)} bytes")
            
            # 원본 데이터도 크기 검증
            expected_data_size = msg.width * msg.height * msg.point_step
            actual_data_size = len(msg.data)
            if actual_data_size != expected_data_size:
                print(f"[DEBUG] Raw data size mismatch! Actual: {actual_data_size}, Expected: {expected_data_size}")
                print(f"[DEBUG] This may cause rviz2 errors")
            
            return msg
            
    except Exception as e:
        print(f"[DEBUG] Draco decoding error: {e}, using raw data")
        # 에러 발생 시 원본 데이터 사용
        msg = PointCloud2()
        msg.header.frame_id = template.header.frame_id
        msg.header.stamp = template.header.stamp
        msg.height = height
        msg.width = width
        msg.is_bigendian = template.is_bigendian
        msg.point_step = point_step
        msg.row_step = point_step * width
        msg.is_dense = template.is_dense
        msg.fields = template.fields
        msg.data = compressed_data  # 원본 데이터 직접 사용
        print(f"[DEBUG] Using raw data due to error: {len(msg.data)} bytes")
        
        # 에러 시 원본 데이터도 크기 검증
        expected_data_size = msg.width * msg.height * msg.point_step
        actual_data_size = len(msg.data)
        if actual_data_size != expected_data_size:
            print(f"[DEBUG] Error raw data size mismatch! Actual: {actual_data_size}, Expected: {expected_data_size}")
            print(f"[DEBUG] This will cause rviz2 errors")
        
        return msg
    
    print(f"[DEBUG] Draco decompression successful: {len(points)} points")
    print(f"[DEBUG] Decoded points shape: {points.shape}")
    print(f"[DEBUG] Decoded data sample (first 3 points): {points[:3].tolist()}")
    print(f"[DEBUG] Decoded data stats - min: {points.min(axis=0)}, max: {points.max(axis=0)}")
    
    # 중복 포인트 제거 로직 제거 - 원본 포인트 순서와 구조를 유지
    # (중복 제거는 포인트 클라우드를 꼬이게 만들 수 있음)
    
    # Draco 해제 성공
    msg = PointCloud2()
    # 완전히 새로운 Header 객체 생성 (frame_id만 복사, stamp는 decoder_client에서 설정)
    from std_msgs.msg import Header
    msg.header = Header()
    msg.header.frame_id = template.header.frame_id
    # stamp는 decoder_client.py에서 현재 시간으로 설정됨
    msg.height = height
    msg.width = width
    msg.is_bigendian = template.is_bigendian
    msg.point_step = point_step
    msg.row_step = point_step * width
    msg.is_dense = template.is_dense
    msg.fields = template.fields
    
    # numpy 배열을 bytes로 변환
    msg.data = points.tobytes()
    print(f"[DEBUG] Converted to {len(msg.data)} bytes")
    
    # 최종 검증
    expected_data_size = msg.width * msg.height * msg.point_step
    actual_data_size = len(msg.data)
    print(f"[DEBUG] Final PointCloud2: width={msg.width}, height={msg.height}, point_step={msg.point_step}")
    print(f"[DEBUG] Final data size: {actual_data_size} bytes, expected: {expected_data_size} bytes")
    
    if actual_data_size != expected_data_size:
        print(f"[DEBUG] ERROR: Data size mismatch! Actual: {actual_data_size}, Expected: {expected_data_size}")
        print(f"[DEBUG] Fixing data size...")
        
        # 데이터 크기를 올바르게 조정
        if actual_data_size > expected_data_size:
            # 데이터가 너무 큰 경우 잘라내기
            msg.data = msg.data[:expected_data_size]
        else:
            # 데이터가 너무 작은 경우 패딩
            padding = b'\x00' * (expected_data_size - actual_data_size)
            msg.data = bytes(msg.data) + padding
        
        print(f"[DEBUG] Fixed data size: {len(msg.data)} bytes")
    
    return msg

def pointcloud2_to_numpy(pc2: PointCloud2) -> np.ndarray:
    """
    PointCloud2를 numpy 배열로 변환
    """
    print(f"[DEBUG] PointCloud2 info: width={pc2.width}, height={pc2.height}, point_step={pc2.point_step}")
    print(f"[DEBUG] PointCloud2 data size: {len(pc2.data)} bytes")
    print(f"[DEBUG] PointCloud2 fields: {[f.name for f in pc2.fields]}")
    
    # 포인트 수 계산
    num_points = pc2.width * pc2.height
    print(f"[DEBUG] Expected points: {num_points}")
    
    # 데이터 타입 결정
    if pc2.point_step == 16:  # x, y, z, intensity (각 4바이트)
        dtype = np.float32
        points_per_row = 4
        print(f"[DEBUG] Using 4-component format (x,y,z,intensity)")
    elif pc2.point_step == 12:  # x, y, z (각 4바이트)
        dtype = np.float32
        points_per_row = 3
        print(f"[DEBUG] Using 3-component format (x,y,z)")
    else:
        # 기본값으로 float32 사용
        dtype = np.float32
        points_per_row = 4
        print(f"[DEBUG] Unknown point_step {pc2.point_step}, using 4-component format")
    
    # 데이터를 numpy 배열로 변환
    data = np.frombuffer(pc2.data, dtype=dtype)
    print(f"[DEBUG] Raw data array shape: {data.shape}")
    
    # 포인트 클라우드 형태로 reshape
    expected_elements = num_points * points_per_row
    print(f"[DEBUG] Expected elements: {expected_elements}, actual elements: {len(data)}")
    
    if len(data) >= expected_elements:
        points = data[:expected_elements].reshape(num_points, points_per_row)
        print(f"[DEBUG] Successfully reshaped to: {points.shape}")
    else:
        # 데이터가 부족한 경우 0으로 패딩
        print(f"[DEBUG] Data insufficient, padding with zeros")
        points = np.zeros((num_points, points_per_row), dtype=dtype)
        available_points = len(data) // points_per_row
        if available_points > 0:
            points[:available_points] = data[:available_points * points_per_row].reshape(available_points, points_per_row)
        print(f"[DEBUG] Final shape after padding: {points.shape}")
    
    # 중복 포인트 체크 및 제거
    unique_points = np.unique(points.view(np.void), axis=0)
    print(f"[DEBUG] Unique points: {len(unique_points)} out of {len(points)} total points")
    if len(unique_points) < len(points):
        duplicate_ratio = (len(points) - len(unique_points)) / len(points) * 100
        print(f"[DEBUG] WARNING: {duplicate_ratio:.2f}% duplicate points detected!")
        print(f"[DEBUG] Removing duplicates to prevent overlapping visualization...")
        
        # 중복 제거 (원본 포인트 순서 유지)
        _, unique_indices = np.unique(points.view(np.void), axis=0, return_index=True)
        points = points[sorted(unique_indices)]
        print(f"[DEBUG] After deduplication: {len(points)} unique points")
    
    return points

# 기존 함수명과의 호환성을 위한 별칭
encode_draco_stub = encode_draco
decode_draco_stub = decode_draco
