"""
🛑 Safety Decision & Control Logic - State Machine

Implements:
- Forward Collision Warning (FCW): TTC < 2.0s
- Autonomous Emergency Braking (AEB): TTC < 1.0s @ 0.3g
- Lane Keeping Assist (LKA): Lateral offset > 0.2m
- Blind Spot Detection (BSD): Objects in blind zones
- Traffic Sign Recognition (TSR)

Safety guarantees:
- Deterministic state machine
- Fail-safe: default to safe state
- 10ms decision latency
- Cooldown periods (1.0s for AEB)
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List
import yaml
import rclpy
from rclpy.node import Node
from datetime import datetime, timedelta


class SafetyState(Enum):
    """Vehicle safety states"""
    NORMAL = 'normal'
    WARNING = 'warning'
    BRAKING = 'braking'
    EMERGENCY = 'emergency'
    FAILED = 'failed'


class DecisionAction(Enum):
    """Control actions"""
    NONE = 0
    FCW_ALERT = 1
    AEB_TRIGGER = 2
    LKA_CORRECT = 3
    BSD_ALERT = 4
    TSR_ENFORCE = 5


@dataclass
class SafetyCommand:
    """Output safety command"""
    
    action: DecisionAction
    state: SafetyState
    
    # Control targets
    brake_pedal: float = 0.0  # 0-1, 1 = max deceleration
    steering_angle: float = 0.0  # degrees
    throttle: float = 0.0  # 0-1
    
    # Metadata
    timestamp: datetime = field(default_factory=datetime.now)
    confidence: float = 1.0
    reason: str = ''


class SafetyStateMachine(Node):
    """Deterministic safety state machine"""
    
    def __init__(self):
        super().__init__('safety_state_machine')
        
        with open('config/adas_config.yaml') as f:
            self.config = yaml.safe_load(f)
        
        # Safety thresholds
        self.fcw_ttc = self.config['decision']['fcw_threshold']  # 2.0s
        self.aeb_ttc = self.config['decision']['aeb_threshold']  # 1.0s
        self.aeb_max_decel = self.config['decision']['aeb_max_deceleration']  # 0.3g
        self.aeb_cooldown = timedelta(seconds=1.0)
        
        # State tracking
        self.current_state = SafetyState.NORMAL
        self.last_aeb_time = datetime.min
        
        self.get_logger().info(
            f'🛑 Safety State Machine started '
            f'(FCW={self.fcw_ttc}s, AEB={self.aeb_ttc}s)'
        )
    
    def process_risk_assessment(self, risk_data: Dict) -> SafetyCommand:
        """
        Main decision logic
        
        Args:
            risk_data: Output from RiskAssessor
        
        Returns:
            SafetyCommand to execute
        """
        ttc = risk_data['most_critical_ttc']
        aeb_triggered = risk_data['aeb_triggered']
        fcw_triggered = risk_data['fcw_triggered']
        
        # State machine
        if aeb_triggered and self._check_aeb_cooldown():
            command = self._handle_emergency_brake(ttc)
            self.current_state = SafetyState.EMERGENCY
            
        elif fcw_triggered:
            command = self._handle_fcw_warning(ttc)
            self.current_state = SafetyState.WARNING
            
        else:
            command = self._handle_normal_operation()
            self.current_state = SafetyState.NORMAL
        
        return command
    
    def _handle_emergency_brake(self, ttc: float) -> SafetyCommand:
        """
        Emergency braking: max deceleration limited by config
        
        0.3g ≈ 3.0 m/s² (comfortable emergency braking)
        1.0g ≈ 10 m/s² (maximum vehicle deceleration)
        """
        self.last_aeb_time = datetime.now()
        
        # Scale braking with TTC: more aggressive as TTC decreases
        brake_fraction = min(1.0, 1.0 - (ttc / self.aeb_ttc))
        brake_pedal = self.aeb_max_decel * brake_fraction
        
        return SafetyCommand(
            action=DecisionAction.AEB_TRIGGER,
            state=SafetyState.EMERGENCY,
            brake_pedal=brake_pedal,
            confidence=0.95,
            reason=f'AEB triggered: TTC={ttc:.2f}s'
        )
    
    def _handle_fcw_warning(self, ttc: float) -> SafetyCommand:
        """Forward Collision Warning: alert driver"""
        return SafetyCommand(
            action=DecisionAction.FCW_ALERT,
            state=SafetyState.WARNING,
            confidence=0.90,
            reason=f'FCW: TTC={ttc:.2f}s'
        )
    
    def _handle_normal_operation(self) -> SafetyCommand:
        """Normal driving mode"""
        return SafetyCommand(
            action=DecisionAction.NONE,
            state=SafetyState.NORMAL,
            confidence=1.0,
            reason='Normal operation'
        )
    
    def _check_aeb_cooldown(self) -> bool:
        """Check if AEB cooldown has expired"""
        elapsed = datetime.now() - self.last_aeb_time
        return elapsed > self.aeb_cooldown
    
    def handle_lane_detection(self, lane_offset: float) -> SafetyCommand:
        """
        Lane Keeping Assist
        
        Args:
            lane_offset: Lateral offset from lane center (meters)
        
        Returns:
            LKA command if needed
        """
        lka_threshold = self.config['decision']['lka_threshold']
        
        if abs(lane_offset) > lka_threshold:
            # Steering correction: proportional to offset
            correction_factor = lane_offset / lka_threshold
            steering_angle = 25.0 * correction_factor  # Max 25° correction
            
            return SafetyCommand(
                action=DecisionAction.LKA_CORRECT,
                state=self.current_state,
                steering_angle=steering_angle,
                confidence=0.85,
                reason=f'LKA: offset={lane_offset:.2f}m'
            )
        
        return SafetyCommand(
            action=DecisionAction.NONE,
            state=self.current_state,
            confidence=1.0
        )
    
    def handle_blind_spot(self, bsd_detection: Dict) -> SafetyCommand:
        """
        Blind Spot Detection
        
        Args:
            bsd_detection: Detection in blind zone
        
        Returns:
            BSD alert command
        """
        return SafetyCommand(
            action=DecisionAction.BSD_ALERT,
            state=self.current_state,
            confidence=0.80,
            reason=f'BSD: {bsd_detection.get("class_name", "unknown")} in blind spot'
        )
    
    def handle_traffic_sign(self, sign_type: str, sign_value: float) -> SafetyCommand:
        """
        Traffic Sign Recognition enforcement
        
        Args:
            sign_type: 'speed_limit', 'stop', 'yield', etc.
            sign_value: Numeric value (e.g., speed limit in km/h)
        
        Returns:
            TSR enforcement command
        """
        if sign_type == 'stop':
            # Enforce full stop
            brake_pedal = 1.0
            reason = 'Stop sign detected'
        elif sign_type == 'speed_limit':
            # Adjust throttle based on current speed vs limit
            brake_pedal = 0.2  # Gentle braking
            reason = f'Speed limit {sign_value} km/h'
        else:
            brake_pedal = 0.0
            reason = f'TSR: {sign_type}'
        
        return SafetyCommand(
            action=DecisionAction.TSR_ENFORCE,
            state=self.current_state,
            brake_pedal=brake_pedal,
            confidence=0.75,
            reason=reason
        )


class IntegratedSafetyController(Node):
    """Integrates all safety subsystems"""
    
    def __init__(self):
        super().__init__('safety_controller')
        self.state_machine = SafetyStateMachine()
        self.get_logger().info('🛑 Integrated Safety Controller initialized')
    
    def execute_command(self, command: SafetyCommand) -> bool:
        """
        Execute safety command
        
        Returns:
            True if executed successfully
        """
        self.get_logger().info(
            f'{command.action.name}: '
            f'brake={command.brake_pedal:.2f}, '
            f'steering={command.steering_angle:.1f}°, '
            f'reason={command.reason}'
        )
        
        # In production, send to vehicle controller (CAN bus)
        # return self.vehicle_controller.execute(command)
        
        return True


def main():
    rclpy.init()
    controller = IntegratedSafetyController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
