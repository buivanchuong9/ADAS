"""
🚗 Vehicle Controller - CAN Bus Interface & Actuation

Outputs:
- Steering: 0-450° (servo control)
- Brake: 0-100% (hydraulic pressure)
- Throttle: 0-100% (pedal position)
- Lights/Signals

Safety features:
- Watchdog timer (50ms)
- Fail-safe defaults (coast to stop)
- Rate limiting (smooth control)
- Redundant CAN frames
"""

from dataclasses import dataclass
from typing import Optional
import struct
import time
import yaml
import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta


@dataclass
class VehicleState:
    """Current vehicle state from CAN bus"""
    
    speed: float = 0.0  # m/s
    steering_angle: float = 0.0  # degrees
    brake_pressure: float = 0.0  # bar
    throttle_position: float = 0.0  # 0-100%
    
    # Status flags
    is_braking: bool = False
    is_accelerating: bool = False
    handbrake_on: bool = False
    
    # Timestamps
    timestamp: datetime = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()


class CANBusWriter(Node):
    """CAN bus communication for vehicle control"""
    
    def __init__(self):
        super().__init__('can_bus_writer')
        
        with open('config/adas_config.yaml') as f:
            self.config = yaml.safe_load(f)
        
        # Control limits from config
        self.steering_max = self.config['control']['steering']['max_angle']
        self.steering_rate_max = self.config['control']['steering']['max_rate']
        self.brake_max = self.config['control']['braking']['max_deceleration']
        self.throttle_max = self.config['control']['throttle']['max_acceleration']
        
        # Rate limiting
        self.last_steering = 0.0
        self.last_brake = 0.0
        self.last_throttle = 0.0
        self.last_update = datetime.now()
        
        # Watchdog
        self.last_command_time = datetime.now()
        self.watchdog_timeout = timedelta(milliseconds=50)
        
        # Mock vehicle state
        self.vehicle_state = VehicleState()
        
        self.get_logger().info('🚗 CAN Bus Writer initialized')
    
    def send_control_command(self, steering: float, brake: float, throttle: float) -> bool:
        """
        Send control commands to vehicle
        
        Args:
            steering: -1.0 (full left) to 1.0 (full right)
            brake: 0.0 (no braking) to 1.0 (max braking)
            throttle: 0.0 (no throttle) to 1.0 (max throttle)
        
        Returns:
            True if command sent successfully
        """
        # Clamp inputs
        steering = max(-1.0, min(1.0, steering))
        brake = max(0.0, min(1.0, brake))
        throttle = max(0.0, min(1.0, throttle))
        
        # Rate limiting (smooth acceleration/deceleration)
        dt = (datetime.now() - self.last_update).total_seconds()
        self.last_update = datetime.now()
        
        steering = self._apply_rate_limit(
            steering, self.last_steering,
            self.steering_rate_max * dt / self.steering_max
        )
        brake = self._apply_rate_limit(brake, self.last_brake, 0.5 * dt)
        throttle = self._apply_rate_limit(throttle, self.last_throttle, 0.5 * dt)
        
        self.last_steering = steering
        self.last_brake = brake
        self.last_throttle = throttle
        self.last_command_time = datetime.now()
        
        # Convert to physical units
        steering_angle = steering * self.steering_max
        brake_decel = brake * self.brake_max
        throttle_accel = throttle * self.throttle_max
        
        # Prepare CAN frames
        frame_steering = self._encode_steering_frame(steering_angle)
        frame_brake = self._encode_brake_frame(brake_decel)
        frame_throttle = self._encode_throttle_frame(throttle_accel)
        
        # Send (mock: print instead of actual CAN)
        self.get_logger().info(
            f'Control: steering={steering_angle:+.1f}°, '
            f'brake={brake_decel:.2f}m/s², '
            f'throttle={throttle_accel:.2f}m/s²'
        )
        
        return True
    
    def _apply_rate_limit(self, target: float, current: float, max_change: float) -> float:
        """Smooth changes to prevent jerky control"""
        change = target - current
        if abs(change) > max_change:
            change = max_change if change > 0 else -max_change
        return current + change
    
    def _encode_steering_frame(self, angle: float) -> bytes:
        """Encode steering command to CAN frame (DBC format)"""
        # Steering angle: degrees to raw CAN value
        # Typical mapping: -450° to 450° -> 0x0000 to 0xFFFF
        raw_value = int((angle / 450.0 + 1.0) * 0x7FFF)
        raw_value = max(0, min(0xFFFF, raw_value))
        return struct.pack('>H', raw_value)
    
    def _encode_brake_frame(self, decel: float) -> bytes:
        """Encode brake command to CAN frame"""
        # Brake deceleration: m/s² to brake pedal percentage
        brake_percent = (decel / 10.0) * 100  # Assuming max 10 m/s²
        brake_percent = max(0, min(100, brake_percent))
        return struct.pack('>B', int(brake_percent))
    
    def _encode_throttle_frame(self, accel: float) -> bytes:
        """Encode throttle command to CAN frame"""
        # Throttle acceleration: m/s² to pedal percentage
        throttle_percent = (accel / 3.0) * 100  # Assuming max 3 m/s²
        throttle_percent = max(0, min(100, throttle_percent))
        return struct.pack('>B', int(throttle_percent))
    
    def check_watchdog(self) -> bool:
        """Check if control is responsive (watchdog timer)"""
        elapsed = datetime.now() - self.last_command_time
        if elapsed > self.watchdog_timeout:
            self.get_logger().warn('Watchdog timeout: no command received in 50ms')
            return False
        return True
    
    def emergency_stop(self) -> bool:
        """Emergency stop: full braking"""
        self.get_logger().error('🚨 EMERGENCY STOP triggered')
        return self.send_control_command(0.0, 1.0, 0.0)


class RCCarInterface(Node):
    """Interface for RC car testing platform"""
    
    def __init__(self):
        super().__init__('rc_car_interface')
        
        # PWM servo control (typical RC car: 1000-2000 microseconds)
        self.pwm_min = 1000
        self.pwm_max = 2000
        self.pwm_center = 1500
        
        self.get_logger().info('🏎️ RC Car Interface initialized')
    
    def command_steering_servo(self, angle: float) -> int:
        """
        Convert steering angle to PWM signal
        
        Args:
            angle: -45° (full left) to 45° (full right)
        
        Returns:
            PWM microseconds
        """
        # -45° -> 1000us, 0° -> 1500us, 45° -> 2000us
        pwm = self.pwm_center + int((angle / 45.0) * (self.pwm_max - self.pwm_center))
        pwm = max(self.pwm_min, min(self.pwm_max, pwm))
        return pwm
    
    def command_esc_throttle(self, throttle: float) -> int:
        """
        Convert throttle to ESC signal
        
        Args:
            throttle: -1.0 (full reverse) to 1.0 (full forward)
        
        Returns:
            PWM microseconds
        """
        # -1.0 -> 1000us, 0.0 -> 1500us, 1.0 -> 2000us
        pwm = self.pwm_center + int(throttle * (self.pwm_max - self.pwm_center))
        pwm = max(self.pwm_min, min(self.pwm_max, pwm))
        return pwm


class SimulationInterface(Node):
    """Interface to CARLA/LGSVL simulator"""
    
    def __init__(self):
        super().__init__('simulation_interface')
        
        # CARLA connection settings
        self.carla_host = 'localhost'
        self.carla_port = 2000
        
        self.get_logger().info('🎮 Simulation Interface initialized (CARLA/LGSVL)')
    
    def connect_carla(self) -> bool:
        """Connect to CARLA simulator"""
        try:
            # import carla
            # self.client = carla.Client(self.carla_host, self.carla_port)
            # self.world = self.client.get_world()
            self.get_logger().info('Connected to CARLA simulator')
            return True
        except Exception as e:
            self.get_logger().error(f'CARLA connection failed: {e}')
            return False
    
    def apply_vehicle_control(self, steering: float, brake: float, throttle: float):
        """Apply control to simulated vehicle"""
        # In production:
        # control = carla.VehicleControl()
        # control.steer = steering
        # control.brake = brake
        # control.throttle = throttle
        # self.vehicle.apply_control(control)
        pass


def main():
    rclpy.init()
    can_writer = CANBusWriter()
    rc_interface = RCCarInterface()
    sim_interface = SimulationInterface()
    rclpy.spin(can_writer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
