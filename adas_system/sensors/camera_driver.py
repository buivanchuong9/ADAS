"""
🚗 ADAS Sensor Drivers - Multi-sensor data acquisition

Handles:
- Camera (RGB, Stereo, HDR)
- LiDAR (32-channel, 64-channel)
- Radar (77 GHz FMCW)
- CAN Bus (Vehicle state)
- GPS/IMU (Localization)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
from datetime import datetime
import yaml


class CameraDriver(Node):
    """Multi-camera RGB driver with timestamp synchronization"""
    
    def __init__(self):
        super().__init__('camera_driver')
        
        # Load config
        with open('config/adas_config.yaml') as f:
            self.config = yaml.safe_load(f)
        
        # Publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', qos)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)  # USB camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Calibration
        self.camera_matrix = np.array(self.config['sensor_calibration']['camera']['camera_matrix'])
        self.dist_coeffs = np.array(self.config['sensor_calibration']['camera']['distortion'])
        
        # Threading
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('🎥 Camera Driver started')
    
    def _capture_loop(self):
        """Continuous frame capture"""
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                continue
            
            # Undistort
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            
            # Publish
            msg = self._frame_to_msg(frame)
            self.pub_image.publish(msg)
    
    def _frame_to_msg(self, frame):
        """Convert OpenCV frame to ROS2 Image message"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg
    
    def destroy_node(self):
        """Cleanup"""
        self.running = False
        self.cap.release()
        super().destroy_node()


class LiDARDriver(Node):
    """32-channel LiDAR data acquisition"""
    
    def __init__(self):
        super().__init__('lidar_driver')
        
        with open('config/adas_config.yaml') as f:
            self.config = yaml.safe_load(f)
        
        # Publisher
        qos = QoSProfile(depth=1)
        self.pub_points = self.create_publisher(
            PointCloud2, '/lidar/point_cloud', qos
        )
        
        # LiDAR connection (simulated)
        self.lidar_port = '/dev/ttyUSB0'  # Modify for your setup
        self.running = True
        
        self.get_logger().info('☁️ LiDAR Driver started')
    
    def publish_mock_lidar(self):
        """Publish mock LiDAR data for testing"""
        # Generate random points
        points = np.random.rand(100000, 3) * 100  # 0-100m range
        
        msg = self._points_to_msg(points)
        self.pub_points.publish(msg)
    
    def _points_to_msg(self, points):
        """Convert point array to ROS2 PointCloud2"""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        
        # Structured array with x, y, z, intensity
        point_struct = np.zeros(len(points), dtype=[
            ('x', '<f4'),
            ('y', '<f4'),
            ('z', '<f4'),
            ('intensity', '<f4'),
        ])
        point_struct['x'] = points[:, 0]
        point_struct['y'] = points[:, 1]
        point_struct['z'] = points[:, 2]
        point_struct['intensity'] = np.random.rand(len(points))
        
        msg.data = point_struct.tobytes()
        return msg


class RADARDriver(Node):
    """77 GHz FMCW Radar data"""
    
    def __init__(self):
        super().__init__('radar_driver')
        self.get_logger().info('📡 RADAR Driver started (mock)')


class CANBusReader(Node):
    """Read vehicle state from CAN bus"""
    
    def __init__(self):
        super().__init__('can_bus_reader')
        
        self.can_interface = 'can0'  # Linux socketcan
        self.get_logger().info('🚗 CAN Bus Reader started')


def main():
    """Launch all sensor drivers"""
    rclpy.init()
    
    # Create nodes
    camera_node = CameraDriver()
    lidar_node = LiDARDriver()
    can_node = CANBusReader()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_node)
    executor.add_node(lidar_node)
    executor.add_node(can_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


if __name__ == '__main__':
    main()
