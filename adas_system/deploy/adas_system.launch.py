"""
🚀 ADAS ROS2 Launch - Orchestrate all 11 modules

Launch architecture:
1. Core: ROS2 daemon
2. Sensors: Camera, LiDAR, Radar, CAN, GPS/IMU
3. Perception: YOLOv8 detector, depth estimator, lane detector
4. Tracking: DeepSORT multi-object tracker
5. Fusion: Multi-modal sensor fusion
6. Prediction: Trajectory prediction + risk assessment
7. Decision: Safety state machine
8. Control: Vehicle controller (CAN/RC/Sim)
9. Utilities: Data logger, visualizer

Total: 15-20 nodes
Cycle time: 30 Hz (33.3ms)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate comprehensive ADAS launch description"""
    
    # Configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_visualization = LaunchConfiguration('enable_visualization', default='true')
    use_carla = LaunchConfiguration('use_carla', default='false')
    
    ld = LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
        DeclareLaunchArgument('enable_visualization', default_value='true',
                            description='Enable RViz visualization'),
        DeclareLaunchArgument('use_carla', default_value='false',
                            description='Use CARLA simulator'),
    ])
    
    # ============ TIER 1: CORE INFRASTRUCTURE ============
    # ROS2 daemon (already running in background)
    
    # ============ TIER 2: SENSOR DRIVERS ============
    camera_driver = Node(
        package='adas_system',
        executable='camera_driver',
        name='camera_driver',
        parameters=[{'device_id': 0}],
    )
    
    lidar_driver = Node(
        package='adas_system',
        executable='lidar_driver',
        name='lidar_driver',
        parameters=[{'port': '/dev/ttyUSB0'}],
    )
    
    radar_driver = Node(
        package='adas_system',
        executable='radar_driver',
        name='radar_driver',
        parameters=[{'can_interface': 'can0'}],
    )
    
    can_bus_reader = Node(
        package='adas_system',
        executable='can_bus_reader',
        name='can_bus_reader',
    )
    
    gps_imu = Node(
        package='adas_system',
        executable='gps_imu_driver',
        name='gps_imu_driver',
        parameters=[{'gps_port': '/dev/ttyUSB1'}],
    )
    
    # ============ TIER 3: PERCEPTION ============
    yolov8_detector = Node(
        package='adas_system',
        executable='yolov8_detector',
        name='yolov8_detector',
        parameters=[{
            'model_path': 'models/yolov8m.pt',
            'conf_threshold': 0.5,
        }],
    )
    
    depth_estimator = Node(
        package='adas_system',
        executable='depth_estimator',
        name='depth_estimator',
        parameters=[{'model_path': 'models/monodepth2.onnx'}],
    )
    
    lane_detector = Node(
        package='adas_system',
        executable='lane_detector',
        name='lane_detector',
        parameters=[{'model_path': 'models/lanenet.onnx'}],
    )
    
    bev_transformer = Node(
        package='adas_system',
        executable='bev_transformer',
        name='bev_transformer',
        parameters=[{'model_path': 'models/bevformer.onnx'}],
    )
    
    lidar_detector = Node(
        package='adas_system',
        executable='lidar_detector',
        name='lidar_detector',
        parameters=[{
            'model_path': 'models/pointpillars.onnx',
            'conf_threshold': 0.3,
        }],
    )
    
    # ============ TIER 4: TRACKING ============
    deepsort_tracker = Node(
        package='adas_system',
        executable='deepsort_tracker',
        name='deepsort_tracker',
        parameters=[{
            'max_age': 30,
            'min_hits': 3,
        }],
    )
    
    # ============ TIER 5: FUSION ============
    sensor_fusion = Node(
        package='adas_system',
        executable='sensor_fusion',
        name='sensor_fusion',
        parameters=[{'fusion_method': 'bevfusion'}],
    )
    
    # ============ TIER 6: PREDICTION ============
    trajectory_predictor = Node(
        package='adas_system',
        executable='trajectory_predictor',
        name='trajectory_predictor',
        parameters=[{
            'horizon_seconds': 3.0,
            'prediction_steps': 30,
        }],
    )
    
    risk_assessor = Node(
        package='adas_system',
        executable='risk_assessor',
        name='risk_assessor',
    )
    
    # ============ TIER 7: DECISION ============
    safety_state_machine = Node(
        package='adas_system',
        executable='safety_state_machine',
        name='safety_state_machine',
        parameters=[{
            'fcw_ttc_threshold': 2.0,
            'aeb_ttc_threshold': 1.0,
        }],
    )
    
    # ============ TIER 8: CONTROL ============
    vehicle_controller = Node(
        package='adas_system',
        executable='vehicle_controller',
        name='vehicle_controller',
    )
    
    # ============ TIER 9: UTILITIES ============
    data_logger = Node(
        package='adas_system',
        executable='data_logger',
        name='data_logger',
        parameters=[{'output_dir': './data/logs'}],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=LaunchConfiguration('enable_visualization'),
    )
    
    # ============ ADD ALL NODES TO LAUNCH ============
    # Tier 2: Sensors
    ld.add_action(camera_driver)
    ld.add_action(lidar_driver)
    ld.add_action(radar_driver)
    ld.add_action(can_bus_reader)
    ld.add_action(gps_imu)
    
    # Tier 3: Perception
    ld.add_action(yolov8_detector)
    ld.add_action(depth_estimator)
    ld.add_action(lane_detector)
    ld.add_action(bev_transformer)
    ld.add_action(lidar_detector)
    
    # Tier 4: Tracking
    ld.add_action(deepsort_tracker)
    
    # Tier 5: Fusion
    ld.add_action(sensor_fusion)
    
    # Tier 6: Prediction
    ld.add_action(trajectory_predictor)
    ld.add_action(risk_assessor)
    
    # Tier 7: Decision
    ld.add_action(safety_state_machine)
    
    # Tier 8: Control
    ld.add_action(vehicle_controller)
    
    # Tier 9: Utilities
    ld.add_action(data_logger)
    ld.add_action(rviz_node)
    
    return ld


# Quick launch notes:
# 
# ros2 launch adas_system adas_system.launch.py
# 
# With visualization:
# ros2 launch adas_system adas_system.launch.py enable_visualization:=true
# 
# With CARLA simulator:
# ros2 launch adas_system adas_system.launch.py use_carla:=true
# 
# Monitor topics:
# ros2 topic list
# ros2 topic echo /perception/camera/detections
# ros2 topic echo /tracking/tracks
# ros2 topic echo /safety/state
# 
# View ROS graph:
# rqt_graph
# 
# Record data:
# ros2 bag record /camera/image_raw /lidar/point_cloud /tracking/tracks /safety/state
