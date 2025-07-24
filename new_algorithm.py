"""
Sine Wave Detection Algorithm for Drone Takeoff
Compatible with QuecPython and existing IMU Handler
"""

import utime
import math
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
        
        # Thresholds (adjust based on your IMU and environment)
        self.IDLE_THRESH = 0.1       # g for accelerometer
        self.LARGE_THRESH = 1.5      # g for accelerometer - INCREASED from 0.5 to 1.5
        self.MOTOR_RIBBLE_THRESH = 0.4  # g for accelerometer - INCREASED from 0.2 to 0.4
        self.GYRO_RIBBLE_THRESH = 5.0   # dps for gyro - INCREASED from 1.0 to 5.0
        self.GYRO_LARGE_THRESH = 100.0   # dps for gyro large disturbance detection
        self.MARGIN = 0.05            # g for trend detection
        self.TREND_THRESHOLD = 5      # out of 10 samples - REDUCED from 7 to 5
        self.WINDOW_SIZE = 8          # REDUCED from 10 to 8 for faster response
        
        # Initialize state and data structures
        self.state = self.STATE_IDLE
        self.accz_window = []  # Using simple list instead of deque
        self.sample_count = 0
        self.state_change_count = 0
    
    def reset(self):
        """Reset detector to idle state"""
        self.state = self.STATE_IDLE
        self.accz_window = []
        print("RESET: Detector reset to IDLE state")
    
    def is_gradual_trend(self, window, direction):
        """Detect gradual trend in a window of AccZ values"""
        if len(window) < 2:
            return False
        
        # Check if the amplitude is sufficient (minimum 0.02g peak-to-peak)
        amplitude = max(window) - min(window)
        if amplitude < 0.02:  # Minimum amplitude threshold - REDUCED from 0.05 to 0.02
            return False
            
        count = 0
        for i in range(1, len(window)):
            if direction == 'rising':
                if window[i] > window[i-1] - self.MARGIN:
                    count += 1
            elif direction == 'falling':
                if window[i] < window[i-1] + self.MARGIN:
                    count += 1
        return count >= self.TREND_THRESHOLD
    
    def other_axes_ok(self, sample):
        """Check if non-Z axes are within ripple thresholds"""
        return (abs(sample['ax']) <= self.MOTOR_RIBBLE_THRESH and
                abs(sample['ay']) <= self.MOTOR_RIBBLE_THRESH and
                abs(sample['gx']) <= self.GYRO_RIBBLE_THRESH and
                abs(sample['gy']) <= self.GYRO_RIBBLE_THRESH and
                abs(sample['gz']) <= self.GYRO_RIBBLE_THRESH)
    
    def large_threshold_exceeded(self, sample):
        """Check if any axis exceeds large threshold"""
        # Separate thresholds for accelerometer (g) and gyroscope (dps)
        accel_exceeded = (abs(sample['ax']) > self.LARGE_THRESH or
                abs(sample['ay']) > self.LARGE_THRESH or
                         abs(sample['az']) > self.LARGE_THRESH)
        
        gyro_exceeded = (abs(sample['gx']) > self.GYRO_LARGE_THRESH or
                        abs(sample['gy']) > self.GYRO_LARGE_THRESH or
                        abs(sample['gz']) > self.GYRO_LARGE_THRESH)
        
        exceeded = accel_exceeded or gyro_exceeded
        
        # Debug: Print what caused the reset
        if exceeded:
            causes = []
            if abs(sample['ax']) > self.LARGE_THRESH:
                causes.append("AX={:.3f}g>{:.1f}g".format(abs(sample['ax']), self.LARGE_THRESH))
            if abs(sample['ay']) > self.LARGE_THRESH:
                causes.append("AY={:.3f}g>{:.1f}g".format(abs(sample['ay']), self.LARGE_THRESH))
            if abs(sample['az']) > self.LARGE_THRESH:
                causes.append("AZ={:.3f}g>{:.1f}g".format(abs(sample['az']), self.LARGE_THRESH))
            if abs(sample['gx']) > self.GYRO_LARGE_THRESH:
                causes.append("GX={:.3f}dps>{:.1f}dps".format(abs(sample['gx']), self.GYRO_LARGE_THRESH))
            if abs(sample['gy']) > self.GYRO_LARGE_THRESH:
                causes.append("GY={:.3f}dps>{:.1f}dps".format(abs(sample['gy']), self.GYRO_LARGE_THRESH))
            if abs(sample['gz']) > self.GYRO_LARGE_THRESH:
                causes.append("GZ={:.3f}dps>{:.1f}dps".format(abs(sample['gz']), self.GYRO_LARGE_THRESH))
            
            print("RESET TRIGGER: {}".format(" | ".join(causes)))
        
        return exceeded
    
    def in_idle_condition(self, sample):
        """Check if all axes are near zero (idle condition)"""
        return (abs(sample['ax']) <= self.IDLE_THRESH and
                abs(sample['ay']) <= self.IDLE_THRESH and
                abs(sample['az']) <= self.IDLE_THRESH and
                abs(sample['gx']) <= self.IDLE_THRESH and
                abs(sample['gy']) <= self.IDLE_THRESH and
                abs(sample['gz']) <= self.IDLE_THRESH)
    
    def update_window(self, value):
        """Update sliding window with new value"""
        self.accz_window.append(value)
        if len(self.accz_window) > self.WINDOW_SIZE:
            # Remove oldest element if window is full
            self.accz_window.pop(0)
    
    def process_sample(self, sample):
        """Process new IMU sample and return current state"""
        self.sample_count += 1
        
        # Reset on large disturbances from any state
        if self.large_threshold_exceeded(sample):
            self.reset()
            return self.state
        
        old_state = self.state
        
        # State machine logic
        if self.state == self.STATE_IDLE:
            if not self.in_idle_condition(sample):
                self.state = self.STATE_MOTOR_ON
                self.accz_window = []
        
        elif self.state == self.STATE_MOTOR_ON:
            if not self.other_axes_ok(sample):
                self.accz_window = []
            else:
                self.update_window(sample['az'])
                if len(self.accz_window) == self.WINDOW_SIZE:
                    amplitude = max(self.accz_window) - min(self.accz_window)
                    if self.is_gradual_trend(self.accz_window, 'rising'):
                        print("[{}] FIRST_RISE detected (amplitude: {:.3f}g, window: {})".format(
                            self.sample_count, amplitude, 
                            [round(x, 3) for x in self.accz_window[-5:]]  # Show last 5 values
                        ))
                        self.state = self.STATE_FIRST_RISE
                        self.accz_window = []
                    elif self.is_gradual_trend(self.accz_window, 'falling'):
                        print("[{}] SECOND_FALL detected (amplitude: {:.3f}g, window: {})".format(
                            self.sample_count, amplitude,
                            [round(x, 3) for x in self.accz_window[-5:]]  # Show last 5 values
                        ))
                        self.state = self.STATE_SECOND_FALL
                        self.accz_window = []
        
        elif self.state == self.STATE_FIRST_RISE:
            if not self.other_axes_ok(sample):
                self.accz_window = []
            else:
                self.update_window(sample['az'])
                if len(self.accz_window) == self.WINDOW_SIZE:
                    amplitude = max(self.accz_window) - min(self.accz_window)
                    if self.is_gradual_trend(self.accz_window, 'falling'):
                        print("[{}] FIRST_FALL detected (amplitude: {:.3f}g, window: {})".format(
                            self.sample_count, amplitude,
                            [round(x, 3) for x in self.accz_window[-5:]]  # Show last 5 values
                        ))
                        self.state = self.STATE_FIRST_FALL
                        self.accz_window = []
        
        elif self.state == self.STATE_FIRST_FALL:
            # Transition immediately to motor on state
            self.state = self.STATE_MOTOR_ON
            self.accz_window = []
        
        elif self.state == self.STATE_SECOND_FALL:
            if not self.other_axes_ok(sample):
                self.accz_window = []
            else:
                self.update_window(sample['az'])
                if len(self.accz_window) == self.WINDOW_SIZE:
                    amplitude = max(self.accz_window) - min(self.accz_window)
                    if self.is_gradual_trend(self.accz_window, 'rising'):
                        print("[{}] SECOND_RISE detected (amplitude: {:.3f}g, window: {})".format(
                            self.sample_count, amplitude,
                            [round(x, 3) for x in self.accz_window[-5:]]  # Show last 5 values
                        ))
                        self.state = self.STATE_SECOND_RISE
                        self.accz_window = []
        
        elif self.state == self.STATE_SECOND_RISE:
            # Transition to steady state
            self.state = self.STATE_STEADY
            self.accz_window = []
        
        elif self.state == self.STATE_STEADY:
            if self.in_idle_condition(sample):
                self.state = self.STATE_IDLE
                self.accz_window = []
        
        # Log state changes
        if old_state != self.state:
            self.state_change_count += 1
            print("[{}] State change: {} -> {}".format(
                self.sample_count, 
                self.get_state_name(old_state), 
                self.get_state_name(self.state)
            ))
        
        return self.state

    def get_state_name(self, state=None):
        """Get human-readable state name"""
        if state is None:
            state = self.state
        names = {
            self.STATE_IDLE: "IDLE",
            self.STATE_MOTOR_ON: "MOTOR_ON",
            self.STATE_FIRST_RISE: "FIRST_HALF_RISE",
            self.STATE_FIRST_FALL: "FIRST_HALF_FALL",
            self.STATE_SECOND_FALL: "SECOND_HALF_FALL",
            self.STATE_SECOND_RISE: "SECOND_HALF_RISE",
            self.STATE_STEADY: "STEADY"
        }
        return names.get(state, "UNKNOWN")
    
    def is_takeoff_detected(self):
        """Check if takeoff sequence is complete"""
        return self.state == self.STATE_STEADY


class SineDetectionSystem:
    """Complete sine detection system with IMU integration"""
    
    def __init__(self, config_manager):
        self.imu_handler = IMUHandler(config_manager)
        self.detector = IMUSineDetector()
        
        # Configure detector thresholds for drone operation
        self.detector.IDLE_THRESH = 0.15       # g
        self.detector.MOTOR_RIBBLE_THRESH = 0.4  # g - INCREASED from 0.25
        self.detector.GYRO_RIBBLE_THRESH = 5.0   # dps - INCREASED from 2.0
        self.detector.GYRO_LARGE_THRESH = 100.0   # dps - NEW: separate large gyro threshold
        self.detector.MARGIN = 0.08             # g
        self.detector.LARGE_THRESH = 1.5        # g - INCREASED from 0.5
        
        print("Sine Detection System initialized")
        print("Thresholds:")
        print("  IDLE: {}g".format(self.detector.IDLE_THRESH))
        print("  MOTOR_RIBBLE: {}g".format(self.detector.MOTOR_RIBBLE_THRESH))
        print("  GYRO_RIBBLE: {}dps".format(self.detector.GYRO_RIBBLE_THRESH))
        print("  GYRO_LARGE: {}dps".format(self.detector.GYRO_LARGE_THRESH))
        print("  ACC_LARGE: {}g".format(self.detector.LARGE_THRESH))
        print("  MARGIN: {}g".format(self.detector.MARGIN))
    
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
        """Get current IMU sample in the format expected by detector"""
        accel_data = self.imu_handler.get_accel()
        gyro_data = self.imu_handler.get_gyro()
        
        # Convert to detector format and apply gravity correction for Z-axis
        return {
            'ax': accel_data['x'],
            'ay': accel_data['y'],
            'az': accel_data['z'] - 1.0,  # Remove gravity
            'gx': gyro_data['x'],
            'gy': gyro_data['y'],
            'gz': gyro_data['z']
        }
    
    def run_detection_loop(self, max_duration_seconds=300, update_rate_ms=100):
        """Run the sine detection loop"""
        print("Starting sine detection loop...")
        print("Looking for sine wave pattern in Z-axis acceleration")
        print("Expected sequence: IDLE -> MOTOR_ON -> FIRST_RISE -> FIRST_FALL -> MOTOR_ON -> SECOND_FALL -> SECOND_RISE -> STEADY")
        
        start_time = utime.time()
        last_debug_time = 0
        
        try:
            while True:
                current_time = utime.time()
                
                # Get IMU sample
                sample = self.get_imu_sample()
                
                # Process sample
                state = self.detector.process_sample(sample)
                
                # Print debug info every 2 seconds
                if current_time - last_debug_time >= 2.0:
                    print("[{}] Current state: {} | Sample: AZ={:.3f} AX={:.3f} AY={:.3f}".format(
                        self.detector.sample_count,
                        self.detector.get_state_name(),
                        sample['az'], sample['ax'], sample['ay']
                    ))
                    last_debug_time = current_time
                
                # Check for takeoff detection
                if self.detector.is_takeoff_detected():
                    print("SUCCESS: TAKEOFF DETECTED! Reached STEADY state")
                    print("Total samples processed: {}".format(self.detector.sample_count))
                    print("State changes: {}".format(self.detector.state_change_count))
                    break
                
                # Check timeout
                if current_time - start_time > max_duration_seconds:
                    print("TIMEOUT: Detection timeout after {} seconds".format(max_duration_seconds))
                    break
                
                # Sleep for update rate
                utime.sleep_ms(update_rate_ms)
                
        except KeyboardInterrupt:
            print("STOP: Detection stopped by user")
        except Exception as e:
            print("ERROR: Error in detection loop: {}".format(e))
        finally:
            self.stop()

    def run_calibration_mode(self, duration_seconds=30):
        """Run calibration mode to observe sensor values and tune thresholds"""
        print("=== CALIBRATION MODE ===")
        print("Observing sensor values for {} seconds...".format(duration_seconds))
        print("Keep the sensor still, then move it gently to see typical values")
        
        start_time = utime.time()
        sample_count = 0
        max_values = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
        min_values = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
        
        try:
            while utime.time() - start_time < duration_seconds:
                sample = self.get_imu_sample()
                sample_count += 1
                
                # Track min/max values
                for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
                    if abs(sample[axis]) > abs(max_values[axis]):
                        max_values[axis] = sample[axis]
                    if abs(sample[axis]) < abs(min_values[axis]) or min_values[axis] == 0:
                        min_values[axis] = sample[axis]
                
                # Print every 2 seconds
                if sample_count % 20 == 0:
                    print("[{}] ACC: X={:.3f} Y={:.3f} Z={:.3f} | GYRO: X={:.3f} Y={:.3f} Z={:.3f}".format(
                        sample_count,
                        sample['ax'], sample['ay'], sample['az'],
                        sample['gx'], sample['gy'], sample['gz']
                    ))
                
                utime.sleep_ms(100)
                
        except KeyboardInterrupt:
            print("Calibration stopped by user")
        
        # Print summary
        print("\n=== CALIBRATION SUMMARY ===")
        print("Samples collected: {}".format(sample_count))
        print("Max absolute values:")
        for axis in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']:
            max_abs = max(abs(max_values[axis]), abs(min_values[axis]))
            print("  {}: {:.3f}".format(axis.upper(), max_abs))
        
        print("\nRecommended thresholds:")
        max_acc = max(abs(max_values['ax']), abs(max_values['ay']), abs(max_values['az']))
        max_gyro = max(abs(max_values['gx']), abs(max_values['gy']), abs(max_values['gz']))
        print("  IDLE_THRESH: {:.2f}g".format(max_acc * 1.5))
        print("  MOTOR_RIBBLE_THRESH: {:.2f}g".format(max_acc * 2.0))
        print("  GYRO_RIBBLE_THRESH: {:.1f}dps".format(max_gyro * 1.5))
        print("  LARGE_THRESH: {:.2f}g".format(max_acc * 3.0))
        
        return max_values, min_values


# Example usage
if __name__ == "__main__":
    try:
        # Import config manager
        from usr.config_manager import ConfigManager
        
        # Create system
        config_mgr = ConfigManager()
        detection_system = SineDetectionSystem(config_mgr)
        
        # Start IMU handler
        if not detection_system.start():
            print("ERROR: Failed to start detection system")
            exit(1)
        
        # Ask user what to do
        print("\n=== CHOOSE MODE ===")
        print("1. Run calibration mode (recommended first)")
        print("2. Run detection mode")
        print("3. Exit")
        
        # For now, run calibration first to tune thresholds
        print("\nRunning calibration mode first...")
        max_vals, min_vals = detection_system.run_calibration_mode(duration_seconds=15)
        
        print("\nStarting detection mode...")
        detection_system.run_detection_loop(max_duration_seconds=60)
            
    except Exception as e:
        print("ERROR: {}".format(e))