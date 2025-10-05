# src/draco_bridge/utils.py
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from .draco_wrapper import encode_pointcloud_with_draco, decode_pointcloud_with_draco

def encode_draco(pc2: PointCloud2) -> bytes:
    """
    PointCloud2를 Draco로 압축
    """
    print(f"[DEBUG] encode_draco called with {len(pc2.data)} bytes")
    
    try:
        # PointCloud2 데이터를 numpy 배열로 변환
        points = pointcloud2_to_numpy(pc2)
        print(f"[DEBUG] Converted to numpy array: {points.shape}")
        
        # Draco로 압축 시도
        compressed_data = encode_pointcloud_with_draco(points)
        
        if compressed_data is not None:
            print(f"[DEBUG] Draco compression successful: {len(compressed_data)} bytes")
            # Draco 압축 성공 시 헤더와 함께 반환
            header = struct.pack('<III', pc2.width, pc2.height, pc2.point_step)
            return header + compressed_data
        else:
            print(f"[DEBUG] Draco compression failed")
        
    except Exception as e:
        print(f"[DEBUG] Draco encoding error: {e}")
    
    # Draco 압축 실패 시 zlib으로 fallback
    print(f"[DEBUG] Falling back to zlib compression")
    import zlib
    raw = bytes(pc2.data)
    comp = zlib.compress(raw, level=6)
    header = struct.pack('<III', pc2.width, pc2.height, pc2.point_step)
    
    print(f"[DEBUG] Zlib encoded: header={len(header)} bytes, compressed={len(comp)} bytes")
    return header + comp

def decode_draco(buf: bytes, template: PointCloud2) -> PointCloud2:
    """
    Draco 압축된 데이터를 PointCloud2로 해제
    """
    print(f"[DEBUG] decode_draco called with {len(buf)} bytes")
    
    try:
        # 헤더 분리
        header_size = 12
        if len(buf) < header_size:
            print(f"[DEBUG] Buffer too small: {len(buf)} < {header_size}")
            return template
            
        width, height, point_step = struct.unpack('<III', buf[:header_size])
        compressed_data = buf[header_size:]
        
        print(f"[DEBUG] Header: width={width}, height={height}, point_step={point_step}")
        print(f"[DEBUG] Compressed data size: {len(compressed_data)}")
        
        # Draco로 해제 시도
        points = decode_pointcloud_with_draco(compressed_data)
        
        if points is not None and len(points) > 0:
            print(f"[DEBUG] Draco decompression successful: {len(points)} points")
            # Draco 해제 성공
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
            
            # numpy 배열을 bytes로 변환
            msg.data = points.tobytes()
            print(f"[DEBUG] Converted to {len(msg.data)} bytes")
            return msg
        else:
            print(f"[DEBUG] Draco decompression failed or empty result")
            
    except Exception as e:
        print(f"Draco decoding error: {e}")
    
    # Draco 해제 실패 시 zlib으로 fallback
    print(f"[DEBUG] Falling back to zlib decompression")
    import zlib
    
    try:
        header_size = 12
        width, height, point_step = struct.unpack('<III', buf[:header_size])
        compressed_data = buf[header_size:]
        
        print(f"[DEBUG] Zlib fallback: width={width}, height={height}, point_step={point_step}")
        print(f"[DEBUG] Zlib compressed data size: {len(compressed_data)}")
        
        raw = zlib.decompress(compressed_data)
        print(f"[DEBUG] Zlib decompression successful: {len(raw)} bytes")
        
    except zlib.error as e:
        print(f"[DEBUG] Zlib decompression failed: {e}")
        return template
    except Exception as e:
        print(f"[DEBUG] Fallback error: {e}")
        return template
    
    # 최종 메시지 생성
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
    msg.data = raw
    print(f"[DEBUG] Final message data size: {len(msg.data)} bytes")
    return msg

def pointcloud2_to_numpy(pc2: PointCloud2) -> np.ndarray:
    """
    PointCloud2를 numpy 배열로 변환
    """
    # 포인트 수 계산
    num_points = pc2.width * pc2.height
    
    # 데이터 타입 결정
    if pc2.point_step == 16:  # x, y, z, intensity (각 4바이트)
        dtype = np.float32
        points_per_row = 4
    elif pc2.point_step == 12:  # x, y, z (각 4바이트)
        dtype = np.float32
        points_per_row = 3
    else:
        # 기본값으로 float32 사용
        dtype = np.float32
        points_per_row = 4
    
    # 데이터를 numpy 배열로 변환
    data = np.frombuffer(pc2.data, dtype=dtype)
    
    # 포인트 클라우드 형태로 reshape
    if len(data) >= num_points * points_per_row:
        points = data[:num_points * points_per_row].reshape(num_points, points_per_row)
    else:
        # 데이터가 부족한 경우 0으로 패딩
        points = np.zeros((num_points, points_per_row), dtype=dtype)
        available_points = len(data) // points_per_row
        points[:available_points] = data[:available_points * points_per_row].reshape(available_points, points_per_row)
    
    return points

# 기존 함수명과의 호환성을 위한 별칭
encode_draco_stub = encode_draco
decode_draco_stub = decode_draco
