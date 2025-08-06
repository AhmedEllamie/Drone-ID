"""
Sine Wave Detection Algorithm for Drone Takeoff
Compatible with QuecPython and existing IMU Handler
OPTIMIZED VERSION - Essential functions only
"""

import utime
from usr.imu_handler import IMUHandler


class IMUSineDetector:
    def __init__(self):
        # State definitions
        self.STATE_IDLE = 0
        self.STATE_MOTOR_ON = 1
        self.STATE_FIRST_RISE = 2
        self.STATE_FIRST_FALL = 3
        self.STATE_SECOND_FALL = 4
        self.STATE_SECOND_RISE = 5
        self.STATE_STEADY = 6
        
        # Optimized thresholds - CALIBRATED FOR SMALL DRONE
        self.IDLE_THRESH = 0.05  # Reduced from 0.09 (more sensitive for motor detection)
        self.LARGE_THRESH = 1.5  # Increased from 0.9 (allow larger takeoff movements)
        self.GYRO_LARGE_THRESH = 300.0  # Keep same for rotation
        self.MARGIN = 0.05  # Reduced from 0.10 (half) - for Z-axis trend detection
        self.MOTOR_TO_RISE_MARGIN = 0.12  # Higher margin for MOTOR_ON to FIRST_RISE transition
        self.WINDOW_SIZE = 3
        self.MIN_AMPLITUDE = 0.05  # Reduced from 0.10 (half) - for Z-axis amplitude
        self.min_samples_before_transition = 3
        
        # Time-based transition thresholds (seconds) - only for sine wave transitions
        self.TRANSITION_TIMEOUT = 5.0  # Max time between specific sine wave states (increased from 0.4s)
        
        # Time conditions for state transitions
        self.IDLE_MIN_TIME = 5.0  # Minimum time in IDLE before motor detection
        self.MOTOR_ON_MIN_TIME = 1.5  # Minimum time in MOTOR_ON before rising trend detection
        
        # State tracking
        self.state = self.STATE_IDLE
        self.accz_window = []
        self.sample_count = 0
        self.state_change_count = 0
        self.state_entry_time = 0
        self.reset_count = 0
        
        # Drone status tracking
        self.drone_status = "STOP"  # "START" or "STOP"
        self.idle_start_time = None
        self.idle_timeout = 10.0  # 10 seconds
        self.landing_check_start = None  # Track landing check time
        self.landing_check_duration = 10.0  # 10 seconds to confirm landing
        
        # Real-time Analytics
        self.analytics = {
            'start_time': utime.time(),
            'total_samples': 0,
            'state_durations': {},  # Track time spent in each state
            'state_transitions': [],  # History of state transitions
            'performance_metrics': {
                'avg_sample_rate': 0,
                'detection_time': 0,
                'false_positive_rate': 0,
                'reset_frequency': 0
            },
            'data_statistics': {
                'accel_stats': {'min': [], 'max': [], 'mean': [], 'std': []},
                'gyro_stats': {'min': [], 'max': [], 'mean': [], 'std': []},
                'trend_analysis': {'rising_count': 0, 'falling_count': 0, 'no_trend_count': 0}
            },
            'threshold_analysis': {
                'motor_detections': 0,
                'large_threshold_exceeded': 0,
                'reset_reasons': {}
            },
            'real_time_alerts': []
        }
        
        # Initialize state durations
        state_names = ["IDLE", "MOTOR_ON", "FIRST_RISE", "FIRST_FALL", "SECOND_FALL", "SECOND_RISE", "STEADY"]
        for state_name in state_names:
            self.analytics['state_durations'][state_name] = 0
    
    def reset(self, reason=None):
        """Reset detector to idle state"""
        self.state = self.STATE_IDLE
        self.accz_window = []
        self.state_entry_time = utime.time()
        self.reset_count += 1
        self.landing_check_start = None  # Reset landing check timer
        
        # Reset drone status tracking
        if self.drone_status == "START":
            self.drone_status = "STOP"
            print("DRONE STATUS: STOP (reset)")
        self.idle_start_time = utime.time()  # Start idle timer from reset
        
        print("RESET #{}: Detector reset to IDLE state".format(self.reset_count))
        if reason:
            print("Reason: {}".format(reason))
    
    def is_simple_trend(self, window, direction):
        """Simplified trend detection for noisy data"""
        if len(window) < 2:
            return False
        
        amplitude = max(window) - min(window)
        if amplitude < self.MIN_AMPLITUDE:
            return False
        
        if direction == 'rising':
            trend_detected = window[-1] > window[0] + self.MARGIN
        elif direction == 'falling':
            trend_detected = window[-1] < window[0] - self.MARGIN
        else:
            trend_detected = False
        
        if trend_detected:
            print("[{}] Trend {} detected: {:.3f} -> {:.3f}, amplitude={:.3f}g".format(
                self.sample_count, direction, window[0], window[-1], amplitude
            ))
        
        return trend_detected
    
    def large_threshold_exceeded(self, sample):
        """Check if any axis exceeds large threshold - more lenient during takeoff"""
        # Use different thresholds based on state
        if self.state in [self.STATE_FIRST_FALL, self.STATE_SECOND_FALL, self.STATE_SECOND_RISE]:
            # During takeoff states, allow larger movements
            takeoff_threshold = 2.0  # Higher threshold during takeoff
            exceeded = (abs(sample['ax']) > takeoff_threshold or
                       abs(sample['ay']) > takeoff_threshold or
                       abs(sample['az']) > takeoff_threshold or
                       abs(sample['gx']) > self.GYRO_LARGE_THRESH or
                       abs(sample['gy']) > self.GYRO_LARGE_THRESH or
                       abs(sample['gz']) > self.GYRO_LARGE_THRESH)
        else:
            # Use normal threshold for other states
            exceeded = (abs(sample['ax']) > self.LARGE_THRESH or
                       abs(sample['ay']) > self.LARGE_THRESH or
                       abs(sample['az']) > self.LARGE_THRESH or
                       abs(sample['gx']) > self.GYRO_LARGE_THRESH or
                       abs(sample['gy']) > self.GYRO_LARGE_THRESH or
                       abs(sample['gz']) > self.GYRO_LARGE_THRESH)
        
        if exceeded:
            print("RESET: Large threshold exceeded - AX={:.2f} AY={:.2f} AZ={:.2f} GX={:.1f} GY={:.1f} GZ={:.1f}".format(
                abs(sample['ax']), abs(sample['ay']), abs(sample['az']),
                abs(sample['gx']), abs(sample['gy']), abs(sample['gz'])
            ))
            # Record analytics
            self.record_large_threshold_exceeded()
            self.add_real_time_alert("THRESHOLD_EXCEEDED", 
                "Large threshold exceeded: AX={:.3f} AY={:.3f} AZ={:.3f} GX={:.1f} GY={:.1f} GZ={:.1f}".format(
                    sample['ax'], sample['ay'], sample['az'], sample['gx'], sample['gy'], sample['gz']), 
                "WARNING")
        
        return exceeded
    
    def in_idle_condition(self, sample):
        """Check if all axes are near zero (idle condition) - Z-axis more sensitive"""
        if sample['az'] < -0.5:  # Calibration artifact
            return True
            
        return (abs(sample['az']) <= self.IDLE_THRESH and
                abs(sample['ax']) <= self.IDLE_THRESH and
                abs(sample['ay']) <= self.IDLE_THRESH and
                abs(sample['gx']) <= 20.0 and
                abs(sample['gy']) <= 20.0 and
                abs(sample['gz']) <= 20.0)
    
    def in_steady_idle_condition(self, sample):
        """More strict idle condition for STEADY -> IDLE transition (landing detection)"""
        if sample['az'] < -0.5:  # Calibration artifact
            return True
            
        # More strict thresholds for landing detection
        STEADY_IDLE_THRESH = 0.03  # Even more sensitive for landing
        STEADY_GYRO_THRESH = 10.0  # Lower gyro threshold for landing
        
        return (abs(sample['az']) <= STEADY_IDLE_THRESH and
                abs(sample['ax']) <= STEADY_IDLE_THRESH and
                abs(sample['ay']) <= STEADY_IDLE_THRESH and
                abs(sample['gx']) <= STEADY_GYRO_THRESH and
                abs(sample['gy']) <= STEADY_GYRO_THRESH and
                abs(sample['gz']) <= STEADY_GYRO_THRESH)
    
    def detect_motor_start(self, sample):
        """More sensitive motor start detection for small drones"""
        # Check for any movement above very low threshold
        movement_threshold = 0.02  # Very low threshold for motor detection
        max_movement_threshold = 0.08  # Max threshold to prevent false triggers
        
        # Check if any axis shows movement within acceptable range
        has_movement = ((abs(sample['az']) > movement_threshold and abs(sample['az']) < max_movement_threshold) or
                       (abs(sample['ax']) > movement_threshold and abs(sample['ax']) < max_movement_threshold) or
                       (abs(sample['ay']) > movement_threshold and abs(sample['ay']) < max_movement_threshold))
        
        # Check for gyro movement (motor vibrations) within acceptable range
        gyro_threshold = 5.0  # Min gyro threshold
        max_gyro_threshold = 15.0  # Max gyro threshold
        
        has_gyro_movement = ((abs(sample['gx']) > gyro_threshold and abs(sample['gx']) < max_gyro_threshold) or
                           (abs(sample['gy']) > gyro_threshold and abs(sample['gy']) < max_gyro_threshold) or
                           (abs(sample['gz']) > gyro_threshold and abs(sample['gz']) < max_gyro_threshold))
        
        if has_movement or has_gyro_movement:
            print("[{}] Motor start detected: AZ={:.3f} AX={:.3f} AY={:.3f} GX={:.1f} GY={:.1f} GZ={:.1f}".format(
                self.sample_count, sample['az'], sample['ax'], sample['ay'],
                sample['gx'], sample['gy'], sample['gz']
            ))
            return True
        
        return False
    
    def update_window(self, value):
        """Update sliding window with new value - Z-axis more sensitive"""
        # More sensitive filtering for Z-axis: allow smaller movements
        if value < -0.5 or abs(value) > 2.0:  # Reduced from 3.0 to 2.0 for Z-axis
            return
            
        self.accz_window.append(value)
        if len(self.accz_window) > self.WINDOW_SIZE:
            self.accz_window.pop(0)

    def process_sample(self, sample):
        """Process new IMU sample and return current state"""
        self.sample_count += 1
        current_time = utime.time()
        
        # Reset on large disturbances
        if self.large_threshold_exceeded(sample):
            self.reset("Large threshold exceeded")
            return self.state
        
        # Check specific reset conditions
        if self.check_reset_conditions(sample):
            return self.state

        old_state = self.state
        self.update_window(sample['az'])
        
        # State machine logic
        if self.state == self.STATE_IDLE:
            if self.sample_count < self.min_samples_before_transition:
                return self.state
            
            # Use more sensitive motor detection
            if self.detect_motor_start(sample):
                # Check if enough time has passed in IDLE state (only if not just reset)
                if self.reset_count == 0 and current_time - self.state_entry_time < self.IDLE_MIN_TIME:
                    # False positive - reset to IDLE
                    self.reset("False positive: Motor detected before minimum IDLE time ({:.1f}s < {:.1f}s)".format(
                        current_time - self.state_entry_time, self.IDLE_MIN_TIME
                    ))
                    return self.state
                
                self.state = self.STATE_MOTOR_ON
                self.state_entry_time = current_time
                self.accz_window = []
        
        elif self.state == self.STATE_MOTOR_ON:
            if self.is_simple_trend(self.accz_window, 'rising'):
                # Check if enough time has passed in MOTOR_ON state
                if current_time - self.state_entry_time < self.MOTOR_ON_MIN_TIME:
                    # False positive - reset to IDLE
                    self.reset("False positive: Rising trend detected before minimum MOTOR_ON time ({:.1f}s < {:.1f}s)".format(
                        current_time - self.state_entry_time, self.MOTOR_ON_MIN_TIME
                    ))
                    return self.state
                
                # Use higher margin for MOTOR_ON to FIRST_RISE transition
                if len(self.accz_window) >= 2:
                    amplitude = max(self.accz_window) - min(self.accz_window)
                    if amplitude >= self.MIN_AMPLITUDE:
                        # Check with higher margin for this specific transition
                        if self.accz_window[-1] > self.accz_window[0] + self.MOTOR_TO_RISE_MARGIN:
                            print("[{}] MOTOR_ON to FIRST_RISE: Using higher margin {:.3f}g - {:.3f} -> {:.3f}".format(
                                self.sample_count, self.MOTOR_TO_RISE_MARGIN, self.accz_window[0], self.accz_window[-1]
                            ))
                            self.state = self.STATE_FIRST_RISE
                            self.state_entry_time = current_time
                            self.accz_window = []
                        else:
                            # Rising trend detected but not strong enough for this transition
                            print("[{}] MOTOR_ON: Rising trend too weak for FIRST_RISE transition ({:.3f} < {:.3f} + {:.3f})".format(
                                self.sample_count, self.accz_window[-1], self.accz_window[0], self.MOTOR_TO_RISE_MARGIN
                            ))
        
        elif self.state == self.STATE_FIRST_RISE:
            # Check timeout for FIRST_RISE → FIRST_FALL transition
            if current_time - self.state_entry_time > self.TRANSITION_TIMEOUT:
                self.reset("FIRST_RISE timeout - no falling trend detected")
                return self.state
                
            if self.is_simple_trend(self.accz_window, 'falling'):
                self.state = self.STATE_FIRST_FALL
                self.state_entry_time = current_time
                self.accz_window = []
        
        elif self.state == self.STATE_FIRST_FALL:
            # Check timeout for FIRST_FALL → SECOND_FALL transition
            if current_time - self.state_entry_time > self.TRANSITION_TIMEOUT:
                self.reset("FIRST_FALL timeout - no second falling trend detected")
                return self.state
                
            if self.is_simple_trend(self.accz_window, 'falling'):
                self.state = self.STATE_SECOND_FALL
                self.state_entry_time = current_time
                self.accz_window = []
        
        elif self.state == self.STATE_SECOND_FALL:
            # Check timeout for SECOND_FALL → SECOND_RISE transition
            if current_time - self.state_entry_time > self.TRANSITION_TIMEOUT:
                self.reset("SECOND_FALL timeout - no rising trend detected")
                return self.state
                
            if self.is_simple_trend(self.accz_window, 'rising'):
                self.state = self.STATE_SECOND_RISE
                self.state_entry_time = current_time
                self.accz_window = []
        
        elif self.state == self.STATE_SECOND_RISE:
            self.state = self.STATE_STEADY
            self.state_entry_time = current_time
            self.accz_window = []
        
        elif self.state == self.STATE_STEADY:
            # Check if we should start landing detection
            if self.in_steady_idle_condition(sample):
                if self.landing_check_start is None:
                    # Start landing check timer
                    self.landing_check_start = current_time
                    print("[{}] Landing check started - monitoring for {} seconds".format(
                        self.sample_count, self.landing_check_duration
                    ))
                elif current_time - self.landing_check_start >= self.landing_check_duration:
                    # Landing confirmed after 10 seconds of idle condition
                    self.state = self.STATE_IDLE
                    self.state_entry_time = current_time
                    self.accz_window = []
                    self.landing_check_start = None  # Reset landing check
                    # Immediately reset drone status to STOP when going to IDLE
                    if self.drone_status == "START":
                        self.drone_status = "STOP"
                        print("DRONE STATUS: STOP (landed after {}s idle check)".format(self.landing_check_duration))
            else:
                # Not in idle condition, reset landing check
                if self.landing_check_start is not None:
                    print("[{}] Landing check cancelled - movement detected".format(self.sample_count))
                    self.landing_check_start = None
        
        # Update drone status
        self.update_drone_status()
        
        # Log state changes
        if old_state != self.state:
            self.state_change_count += 1
            state_names = ["IDLE", "MOTOR_ON", "FIRST_RISE", "FIRST_FALL", "SECOND_FALL", "SECOND_RISE", "STEADY"]
            print("[{}] State: {} -> {}".format(
                self.sample_count, 
                state_names[old_state], 
                state_names[self.state]
            ))
            
            # Check for takeoff detection
            if self.state == self.STATE_STEADY and self.drone_status != "START":
                self.drone_status = "START"
                print("SUCCESS: TAKEOFF DETECTED!")
                print("DRONE STATUS: START")
        
        return self.state

    def get_state_name(self):
        """Get current state name"""
        names = ["IDLE", "MOTOR_ON", "FIRST_RISE", "FIRST_FALL", "SECOND_FALL", "SECOND_RISE", "STEADY"]
        return names[self.state]
    
    def is_takeoff_detected(self):
        """Check if takeoff sequence is complete"""
        return self.state == self.STATE_STEADY

    def update_drone_status(self):
        """Update drone status based on current state and idle time"""
        current_time = utime.time()
        
        if self.state == self.STATE_IDLE:
            # Track idle time
            if self.idle_start_time is None:
                self.idle_start_time = current_time
            elif current_time - self.idle_start_time >= self.idle_timeout:
                # Been idle for 10+ seconds, set status to STOP
                if self.drone_status != "STOP":
                    self.drone_status = "STOP"
                    print("DRONE STATUS: STOP (idle for {:.1f} seconds)".format(
                        current_time - self.idle_start_time
                    ))
        else:
            # Not idle, reset idle timer
            self.idle_start_time = None
    
    def get_drone_status(self):
        """Get current drone status"""
        return self.drone_status

    def check_reset_conditions(self, sample):
        """Check for reset conditions based on current state"""
        # Maximum X/Y movement thresholds for different states
        MAX_XY_STEP2 = 0.8  # Max X/Y in step 2 (ripples)
        MAX_XY_STEP3 = 1.0  # Max X/Y in step 3 (takeoff)
        
        max_xy = max(abs(sample['ax']), abs(sample['ay']))
        
        if self.state == self.STATE_MOTOR_ON or self.state == self.STATE_FIRST_RISE:
            # Check for excessive X/Y movement (manual handling)
            if max_xy > MAX_XY_STEP2:
                self.reset("Excessive X/Y movement in early states: {:.3f}g > {:.1f}g".format(max_xy, MAX_XY_STEP2))
                return True
        
        elif self.state == self.STATE_FIRST_FALL or self.state == self.STATE_SECOND_FALL:
            # Check for excessive X/Y movement during takeoff
            if max_xy > MAX_XY_STEP3:
                self.reset("Excessive X/Y movement during takeoff: {:.3f}g > {:.1f}g".format(max_xy, MAX_XY_STEP3))
                return True
        
        # Check if motors stopped (only in early states, not during flight)
        if self.state == self.STATE_MOTOR_ON or self.state == self.STATE_FIRST_RISE:
            total_movement = abs(sample['ax']) + abs(sample['ay']) + abs(sample['az'])
            if total_movement < 0.005:  # Reduced from 0.01g to 0.005g for motor stop detection
                self.reset("Motors stopped - total movement: {:.3f}g < 0.005g".format(total_movement))
                return True
        
        # Check for excessive rotation (manual handling)
        max_gyro = max(abs(sample['gx']), abs(sample['gy']), abs(sample['gz']))
        if self.state != self.STATE_IDLE and self.state != self.STATE_STEADY:
            if max_gyro > 70.0:  # High rotation threshold
                self.reset("Excessive rotation detected: {:.1f}dps > 100.0dps".format(max_gyro))
                return True
        
        return False


class SineDetectionSystem:
    """Optimized sine detection system"""
    
    def __init__(self, config_manager):
        self.imu_handler = IMUHandler(config_manager)
        self.detector = IMUSineDetector()
        print("Optimized Sine Detection System initialized")
    
    def start(self):
        """Start the IMU handler"""
        if not self.imu_handler.start():
            print("ERROR: Failed to start IMU handler")
            return False
        print("SUCCESS: IMU handler started")
        return True
    
    def stop(self):
        """Stop the IMU handler"""
        self.imu_handler.stop()
        print("STOP: Detection system stopped")
    
    def get_imu_sample(self):
        """Get current IMU sample"""
        accel_data = self.imu_handler.get_accel()
        gyro_data = self.imu_handler.get_gyro()
        return {
            'ax': accel_data['x'],
            'ay': accel_data['y'],
            'az': accel_data['z'] - 1.0,  # Remove gravity
            'gx': gyro_data['x'],
            'gy': gyro_data['y'],
            'gz': gyro_data['z']
        }
    
    def run_detection_loop(self, max_duration_seconds=10, update_rate_ms=10):
        """Run the optimized detection loop"""
        print("Starting optimized sine detection...")
        print("Sequence: IDLE -> MOTOR_ON -> FIRST_RISE -> FIRST_FALL -> SECOND_FALL -> SECOND_RISE -> STEADY")
        
        start_time = utime.time()
        last_debug_time = 0
        
        try:
            while True:
                current_time = utime.time()
                sample = self.get_imu_sample()
                state = self.detector.process_sample(sample)
                
                # Debug output every 5 samples
                if self.detector.sample_count % 5 == 0:
                    print("[{}] State: {} | Status: {} | AZ={:.3f} AX={:.3f} AY={:.3f}".format(
                        self.detector.sample_count,
                        self.detector.get_state_name(),
                        self.detector.get_drone_status(),
                        sample['az'], sample['ax'], sample['ay']
                    ))
                
                # Check for takeoff detection (removed duplicate - now handled in process_sample)
                # Continue monitoring for status changes
                if self.detector.get_drone_status() == "START" and self.detector.is_takeoff_detected():
                    # Takeoff detected - continue monitoring for STOP status
                    if self.detector.sample_count % 20 == 0:  # Print status every 20 samples
                        print("Monitoring: Drone is STARTED - waiting for idle timeout...")
                
                # Check timeout (only if no takeoff detected yet)
                if self.detector.get_drone_status() == "STOP" and current_time - start_time > max_duration_seconds:
                    print("TIMEOUT: No takeoff detected in {} seconds".format(max_duration_seconds))
                    break
                
                utime.sleep_ms(update_rate_ms)
                
        except KeyboardInterrupt:
            print("STOP: Detection stopped by user")
        except Exception as e:
            print("ERROR: {}".format(e))
        finally:
            # Print final summary
            print("\n=== FINAL SUMMARY ===")
            print("Total samples processed: {}".format(self.detector.sample_count))
            print("State changes: {}".format(self.detector.state_change_count))
            print("Reset count: {}".format(self.detector.reset_count))
            print("Final drone status: {}".format(self.detector.get_drone_status()))
            print("Total runtime: {:.2f} seconds".format(utime.time() - start_time))
            self.stop()


# Main execution
#if __name__ == "__main__":
#    try:
#        from usr.config_manager import ConfigManager
#        config_mgr = ConfigManager()
#        detection_system = SineDetectionSystem(config_mgr)
#        
#        if not detection_system.start():
#            print("ERROR: Failed to start detection system")
#            exit(1)
#        
#        print("Starting optimized detection...")
#        detection_system.run_detection_loop(max_duration_seconds=300, update_rate_ms=50)  # 5 minutes monitoring
#            
#    except Exception as e:
#        print("ERROR: {}".format(e))

