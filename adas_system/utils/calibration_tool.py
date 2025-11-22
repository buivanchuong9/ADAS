"""
🔧 Sensor Calibration Tool - Camera & Extrinsics

Calibration procedures:
1. Camera intrinsics (K matrix): Checkerboard pattern
2. Camera distortion: Lens aberration correction
3. LiDAR-Camera extrinsics: 3D-2D correspondence
4. Timestamp synchronization: Frame sync validation
5. Sensor health check: All sensors operational
"""

import cv2
import numpy as np
import yaml
from typing import Tuple, Dict
import rclpy
from rclpy.node import Node


class CameraIntrinsicsCalibrator:
    """Camera intrinsic calibration using checkerboard pattern"""
    
    def __init__(self, checkerboard_size=(9, 6)):
        """
        Args:
            checkerboard_size: (width, height) in squares
        """
        self.checkerboard_size = checkerboard_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Object points (assume 0.025m square size)
        self.objp = np.zeros(
            (checkerboard_size[0] * checkerboard_size[1], 3), np.float32
        )
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= 0.025  # Convert to meters
        
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
    
    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        Detect checkerboard corners
        
        Returns:
            (success, corners)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(
            gray, self.checkerboard_size, None
        )
        
        if ret:
            # Refine corner positions
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.criteria
            )
            self.imgpoints.append(corners2)
            self.objpoints.append(self.objp)
            return True, corners2
        
        return False, None
    
    def calibrate(self, image_shape: Tuple[int, int]) -> Dict:
        """
        Perform calibration
        
        Returns:
            {
                'camera_matrix': K matrix,
                'distortion_coefficients': distortion vector,
                'reprojection_error': RMS error,
                'rvecs': rotation vectors,
                'tvecs': translation vectors,
            }
        """
        if len(self.objpoints) == 0:
            raise ValueError('No calibration images provided')
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_shape,
            None, None
        )
        
        # Calculate reprojection error
        total_error = 0
        total_points = 0
        for i in range(len(self.objpoints)):
            proj_imgpoints, _ = cv2.projectPoints(
                self.objpoints[i], rvecs[i], tvecs[i],
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(self.imgpoints[i], proj_imgpoints, cv2.NORM_L2) / len(proj_imgpoints)
            total_error += error
            total_points += 1
        
        reprojection_error = total_error / total_points
        
        return {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.flatten().tolist(),
            'reprojection_error': reprojection_error,
            'rvecs': [r.flatten().tolist() for r in rvecs],
            'tvecs': [t.flatten().tolist() for t in tvecs],
        }


class ExtrinsicsCalibrator:
    """LiDAR-Camera extrinsic calibration using 3D-2D correspondence"""
    
    @staticmethod
    def calibrate_lidar_camera(
        points_3d: np.ndarray,  # [N, 3] LiDAR points
        points_2d: np.ndarray,  # [N, 2] image points (must be corresponding)
        camera_matrix: np.ndarray,  # [3, 3] K matrix
    ) -> Dict:
        """
        Estimate extrinsic transformation from LiDAR to Camera
        
        Args:
            points_3d: 3D points from LiDAR
            points_2d: 2D points in camera image
            camera_matrix: Camera intrinsic matrix
        
        Returns:
            {
                'rotation': 3x3 rotation matrix (LiDAR to Camera),
                'translation': 3x1 translation vector,
                'reprojection_error': RMS pixel error,
            }
        """
        # Use PnP (Perspective-n-Point) to estimate extrinsics
        _, rvec, tvec = cv2.solvePnP(
            points_3d, points_2d, camera_matrix, None,
            useExtrinsicGuess=False, flags=cv2.SOLVEPNP_EPNP
        )
        
        # Convert rotation vector to matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Reproject and calculate error
        proj_2d, _ = cv2.projectPoints(
            points_3d, rvec, tvec, camera_matrix, None
        )
        reprojection_error = np.linalg.norm(points_2d - proj_2d.reshape(-1, 2))
        
        return {
            'rotation': rotation_matrix.tolist(),
            'translation': tvec.flatten().tolist(),
            'reprojection_error': reprojection_error / len(points_2d),
        }


class TimestampSynchronizer:
    """Synchronize timestamps between multiple sensors"""
    
    def __init__(self, buffer_size=100):
        self.buffer = {}  # topic -> deque of (timestamp, data)
        self.buffer_size = buffer_size
        self.time_offset = {}  # topic -> offset from reference
    
    def register_message(self, topic: str, timestamp: float, data: any):
        """Buffer incoming message"""
        if topic not in self.buffer:
            self.buffer[topic] = []
        
        self.buffer[topic].append((timestamp, data))
        if len(self.buffer[topic]) > self.buffer_size:
            self.buffer[topic].pop(0)
    
    def synchronize(self, reference_topic: str, time_window: float = 0.1) -> Dict:
        """
        Find synchronized messages from all topics
        
        Args:
            reference_topic: Use this topic's timestamps as reference
            time_window: Allow ±time_window seconds for matching
        
        Returns:
            {
                'synchronized': list of dicts with all topic data,
                'time_offsets': {topic: offset in seconds},
            }
        """
        if reference_topic not in self.buffer:
            raise ValueError(f'Reference topic {reference_topic} not in buffer')
        
        synchronized = []
        offsets = {}
        
        for ref_ts, ref_data in self.buffer[reference_topic]:
            sync_frame = {reference_topic: ref_data}
            
            # Find matching messages from other topics
            all_matched = True
            for topic, messages in self.buffer.items():
                if topic == reference_topic:
                    continue
                
                # Find closest message in time
                closest_msg = None
                closest_diff = float('inf')
                
                for msg_ts, msg_data in messages:
                    diff = abs(msg_ts - ref_ts)
                    if diff < closest_diff and diff <= time_window:
                        closest_msg = msg_data
                        closest_diff = diff
                        offsets[topic] = msg_ts - ref_ts
                
                if closest_msg is None:
                    all_matched = False
                    break
                
                sync_frame[topic] = closest_msg
            
            if all_matched:
                synchronized.append(sync_frame)
        
        return {
            'synchronized': synchronized,
            'time_offsets': offsets,
        }


class SensorHealthCheck(Node):
    """Check all sensor connections and health"""
    
    def __init__(self):
        super().__init__('sensor_health_check')
        self.get_logger().info('🔧 Sensor Health Check')
    
    def check_all_sensors(self) -> Dict[str, bool]:
        """
        Check if all sensors are responding
        
        Returns:
            {
                'camera': True/False,
                'lidar': True/False,
                'radar': True/False,
                'can_bus': True/False,
                'gps': True/False,
                'imu': True/False,
            }
        """
        status = {
            'camera': self._check_camera(),
            'lidar': self._check_lidar(),
            'radar': self._check_radar(),
            'can_bus': self._check_can_bus(),
            'gps': self._check_gps(),
            'imu': self._check_imu(),
        }
        
        all_ok = all(status.values())
        self.get_logger().info(
            f'Sensor Health: {status} | Overall: {"✅ OK" if all_ok else "❌ FAILED"}'
        )
        
        return status
    
    def _check_camera(self) -> bool:
        try:
            cap = cv2.VideoCapture(0)
            ret = cap.isOpened()
            cap.release()
            return ret
        except:
            return False
    
    def _check_lidar(self) -> bool:
        # Mock check: would connect to actual LiDAR device
        self.get_logger().info('  - LiDAR: (mock check)')
        return True
    
    def _check_radar(self) -> bool:
        self.get_logger().info('  - Radar: (mock check)')
        return True
    
    def _check_can_bus(self) -> bool:
        self.get_logger().info('  - CAN Bus: (mock check)')
        return True
    
    def _check_gps(self) -> bool:
        self.get_logger().info('  - GPS: (mock check)')
        return True
    
    def _check_imu(self) -> bool:
        self.get_logger().info('  - IMU: (mock check)')
        return True


def save_calibration(calibration_data: Dict, config_file: str):
    """Save calibration results to YAML config"""
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    config['sensor_calibration'].update(calibration_data)
    
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f'✅ Calibration saved to {config_file}')


def main():
    """Interactive calibration workflow"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ADAS Sensor Calibration Tool')
    parser.add_argument('--mode', choices=['camera', 'extrinsics', 'sync', 'health'],
                       default='camera', help='Calibration mode')
    parser.add_argument('--config', default='config/adas_config.yaml',
                       help='Configuration file path')
    
    args = parser.parse_args()
    
    if args.mode == 'camera':
        print('🎥 Camera Intrinsics Calibration')
        print('Instructions:')
        print('  1. Press SPACE to capture frames with checkerboard')
        print('  2. Press Q when done (need 20+ frames)')
        print('  3. Calibration results will be saved')
        
        calibrator = CameraIntrinsicsCalibrator()
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            success, corners = calibrator.detect_checkerboard(frame)
            
            # Visualize
            if success:
                cv2.drawChessboardCorners(frame, calibrator.checkerboard_size, corners, success)
                cv2.putText(frame, f'Frames: {len(calibrator.objpoints)}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Camera Calibration', frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                print(f'Captured frame {len(calibrator.objpoints)}')
            elif key == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        if len(calibrator.objpoints) >= 20:
            print('Computing calibration...')
            result = calibrator.calibrate((1920, 1080))
            print(f'Reprojection error: {result["reprojection_error"]:.4f} pixels')
            save_calibration(result, args.config)
    
    elif args.mode == 'health':
        rclpy.init()
        health_check = SensorHealthCheck()
        health_check.check_all_sensors()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
