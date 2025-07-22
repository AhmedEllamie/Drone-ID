"""
Sine Wave Detection Algorithm for Drone Takeoff
Compatible with QuecPython and existing IMU Handler
FINAL VERSION - Optimized for real drone takeoff detection
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
        
        # Thresholds optimized for real drone takeoff based on log analysis
        self.IDLE_THRESH = 0.2       # g - Increased to handle motor vibrations
        self.LARGE_THRESH = 2.0      # g - Increased to avoid resets during takeoff
        self.MOTOR_RIBBLE_THRESH = 0.4  # g - Reduced to be more sensitive to other axes
        self.GYRO_RIBBLE_THRESH = 50.0   # dps - INCREASED to handle noisy gyro data
        self.GYRO_LARGE_THRESH = 300.0   # dps - Significantly increased for takeoff maneuvers
        self.MARGIN = 0.10            # g - MUCH HIGHER to ignore noise
        self.TREND_THRESHOLD = 2      # out of 3 samples - REDUCED for faster detection
        self.WINDOW_SIZE = 3          # Increased to 4 for more stable detection
        self.MIN_AMPLITUDE = 0.10     # g - INCREASED to ignore noise
        
        # Initialize state and data structures
        self.state = self.STATE_IDLE
        self.accz_window = []
        self.sample_count = 0
        self.state_change_count = 0
        
        # Add timing for state transitions
        self.state_entry_time = 0
        self.min_state_duration = 0.1  # Minimum time to stay in a state (seconds)
        self.min_samples_before_transition = 3  # Wait 30 samples before allowing transitions
    
    def reset(self):
        """Reset detector to idle state"""
        self.state = self.STATE_IDLE
        self.accz_window = []
        self.state_entry_time = utime.time()
        print("RESET: Detector reset to IDLE state")
    
    def is_gradual_trend(self, window, direction):
        """Detect gradual trend in a window of AccZ values"""
        if len(window) < self.WINDOW_SIZE:
            return False
        
        # Check if the amplitude is sufficient for a real signal
        amplitude = max(window) - min(window)
        if amplitude < self.MIN_AMPLITUDE:
            return False
            
        count = 0
        for i in range(1, len(window)):
            if direction == 'rising':
                if window[i] > window[i-1] - self.MARGIN:
                    count += 1
            elif direction == 'falling':
                if window[i] < window[i-1] + self.MARGIN:
                    count += 1
        
        trend_detected = count >= self.TREND_THRESHOLD
        
        # Debug output for trend detection - only print when state actually changes
        # if trend_detected:
        #     print("[{}] Trend {} detected: amplitude={:.3f}g, count={}/{}, window={}".format(
        #         self.sample_count, direction, amplitude, count, len(window)-1,
        #         [round(x, 3) for x in window]
        #     ))
        
        return trend_detected
    
    def is_simple_trend(self, window, direction):
        """Simplified trend detection for noisy data - just check if last value follows direction"""
        if len(window) < 2:
            return False
        
        # Check if the amplitude is sufficient for a real signal
        amplitude = max(window) - min(window)
        if amplitude < self.MIN_AMPLITUDE:
            return False
        
        # Simple check: is the last value in the right direction compared to first?
        if direction == 'rising':
            trend_detected = window[-1] > window[0] + self.MARGIN
        elif direction == 'falling':
            trend_detected = window[-1] < window[0] - self.MARGIN
        else:
            trend_detected = False
        
        if trend_detected:
            print("[{}] Simple trend {} detected: {:.3f} -> {:.3f}, amplitude={:.3f}g".format(
                self.sample_count, direction, window[0], window[-1], amplitude
            ))
        
        return trend_detected
    
    def other_axes_ok(self, sample):
        """Check if non-Z axes are within ripple thresholds"""
        ax_ok = abs(sample['ax']) <= self.MOTOR_RIBBLE_THRESH
        ay_ok = abs(sample['ay']) <= self.MOTOR_RIBBLE_THRESH
        gx_ok = abs(sample['gx']) <= self.GYRO_RIBBLE_THRESH
        gy_ok = abs(sample['gy']) <= self.GYRO_RIBBLE_THRESH
        gz_ok = abs(sample['gz']) <= self.GYRO_RIBBLE_THRESH
        
        all_ok = ax_ok and ay_ok and gx_ok and gy_ok and gz_ok
        
        # Debug output when axes are not OK
        if not all_ok and self.sample_count % 10 == 0:  # Print every 10 samples to avoid spam
            violations = []
            if not ax_ok: violations.append("AX={:.3f}>{:.2f}".format(abs(sample['ax']), self.MOTOR_RIBBLE_THRESH))
            if not ay_ok: violations.append("AY={:.3f}>{:.2f}".format(abs(sample['ay']), self.MOTOR_RIBBLE_THRESH))
            if not gx_ok: violations.append("GX={:.3f}>{:.1f}".format(abs(sample['gx']), self.GYRO_RIBBLE_THRESH))
            if not gy_ok: violations.append("GY={:.3f}>{:.1f}".format(abs(sample['gy']), self.GYRO_RIBBLE_THRESH))
            if not gz_ok: violations.append("GZ={:.3f}>{:.1f}".format(abs(sample['gz']), self.GYRO_RIBBLE_THRESH))
            print("[{}] Other axes violations: {}".format(self.sample_count, " | ".join(violations)))
        
        return all_ok
    
    def large_threshold_exceeded(self, sample):
        """Check if any axis exceeds large threshold"""
        accel_exceeded = (
            abs(sample['ax']) > self.LARGE_THRESH or
            abs(sample['ay']) > self.LARGE_THRESH or
            abs(sample['az']) > self.LARGE_THRESH
        )
        
        gyro_exceeded = (
            abs(sample['gx']) > self.GYRO_LARGE_THRESH or
            abs(sample['gy']) > self.GYRO_LARGE_THRESH or
            abs(sample['gz']) > self.GYRO_LARGE_THRESH
        )
        
        exceeded = accel_exceeded or gyro_exceeded
        
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
        """Check if all axes are near zero (idle condition) - improved for calibration artifacts"""
        # For Z-axis, check if it's close to 0 (gravity compensated)
        # Also check for the -1.0 calibration artifact
        if sample['az'] < -0.5:  # Clearly a calibration artifact
            return True
            
        az_idle = abs(sample['az']) <= self.IDLE_THRESH
        ax_idle = abs(sample['ax']) <= self.IDLE_THRESH
        ay_idle = abs(sample['ay']) <= self.IDLE_THRESH
        gx_idle = abs(sample['gx']) <= 20.0  # More strict for gyro
        gy_idle = abs(sample['gy']) <= 20.0
        gz_idle = abs(sample['gz']) <= 20.0
        
        all_idle = az_idle and ax_idle and ay_idle and gx_idle and gy_idle and gz_idle
        
        # Debug output for idle check
        if self.sample_count % 20 == 0:
            print("[{}] Idle check: AZ={:.3f}({}) AX={:.3f}({}) AY={:.3f}({}) GX={:.1f}({}) = {}".format(
                self.sample_count, sample['az'], az_idle, sample['ax'], ax_idle, 
                sample['ay'], ay_idle, sample['gx'], gx_idle, all_idle
            ))
        
        return all_idle
    
    def update_window(self, value):
        """Update sliding window with new value - filter out calibration artifacts"""
        # Skip obvious calibration artifacts
        if value < -0.5 or abs(value) > 3.0:
            return
            
        self.accz_window.append(value)
        if len(self.accz_window) > self.WINDOW_SIZE:
            self.accz_window.pop(0)
    
    def time_in_current_state(self):
        """Get time spent in current state"""
        return utime.time() - self.state_entry_time
    
    def process_sample(self, sample):
        """Process new IMU sample and return current state"""
        self.sample_count += 1
        
        # Reset on large disturbances from any state
        if self.large_threshold_exceeded(sample):
            self.reset()
            return self.state
        
        old_state = self.state
        self.update_window(sample['az'])
        
        # State machine logic with improved transitions
        if self.state == self.STATE_IDLE:
            # Wait for minimum samples to settle after calibration
            if self.sample_count < self.min_samples_before_transition:
                return self.state
                
            if not self.in_idle_condition(sample):
                self.state = self.STATE_MOTOR_ON
                self.state_entry_time = utime.time()
                self.accz_window = []
        
        elif self.state == self.STATE_MOTOR_ON:
            # Look for the first rise in Z-axis acceleration
            # Debug: show window contents every 10 samples
            if self.sample_count % 10 == 0 and len(self.accz_window) >= 2:
                print("[{}] MOTOR_ON window: {} (looking for rising trend)".format(
                    self.sample_count, [round(x, 3) for x in self.accz_window]
                ))
            
            if self.is_simple_trend(self.accz_window, 'rising'):
                self.state = self.STATE_FIRST_RISE
                self.state_entry_time = utime.time()
                self.accz_window = []
            elif not self.other_axes_ok(sample):
                # Clear window if other axes are too active
                self.accz_window = []
        
        elif self.state == self.STATE_FIRST_RISE:
            # Look for the fall after the rise
            if self.is_simple_trend(self.accz_window, 'falling'):
                self.state = self.STATE_FIRST_FALL
                self.state_entry_time = utime.time()
                self.accz_window = []
            elif not self.other_axes_ok(sample):
                self.accz_window = []
        
        elif self.state == self.STATE_FIRST_FALL:
            # Look for the second fall (below baseline)
            if self.is_simple_trend(self.accz_window, 'falling'):
                self.state = self.STATE_SECOND_FALL
                self.state_entry_time = utime.time()
                self.accz_window = []
            elif not self.other_axes_ok(sample):
                self.accz_window = []
        
        elif self.state == self.STATE_SECOND_FALL:
            # Look for the final rise back to baseline
            if self.is_simple_trend(self.accz_window, 'rising'):
                self.state = self.STATE_SECOND_RISE
                self.state_entry_time = utime.time()
                self.accz_window = []
            elif not self.other_axes_ok(sample):
                self.accz_window = []
        
        elif self.state == self.STATE_SECOND_RISE:
            # Complete sine wave detected
            self.state = self.STATE_STEADY
            self.state_entry_time = utime.time()
            self.accz_window = []
        
        elif self.state == self.STATE_STEADY:
            # Stay in steady state for a while, then check for return to idle
            if self.time_in_current_state() > 2.0 and self.in_idle_condition(sample):
                self.state = self.STATE_IDLE
                self.state_entry_time = utime.time()
                self.accz_window = []
        
        # Log state changes
        if old_state != self.state:
            self.state_change_count += 1
            print("[{}] State change: {} -> {} (time in prev state: {:.2f}s)".format(
                self.sample_count, 
                self.get_state_name(old_state), 
                self.get_state_name(self.state),
                utime.time() - self.state_entry_time if old_state != self.STATE_IDLE else 0
            ))
        
        return self.state

    def get_state_name(self, state=None):
        """Get human-readable state name"""
        if state is None:
            state = self.state
        names = {
            self.STATE_IDLE: "IDLE",
            self.STATE_MOTOR_ON: "MOTOR_ON",
            self.STATE_FIRST_RISE: "FIRST_RISE",
            self.STATE_FIRST_FALL: "FIRST_FALL",
            self.STATE_SECOND_FALL: "SECOND_FALL",
            self.STATE_SECOND_RISE: "SECOND_RISE",
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
        
        print("Sine Detection System initialized")
        print("Optimized thresholds for real drone takeoff:")
        print("  IDLE: {}g".format(self.detector.IDLE_THRESH))
        print("  MOTOR_RIBBLE: {}g".format(self.detector.MOTOR_RIBBLE_THRESH))
        print("  GYRO_RIBBLE: {}dps".format(self.detector.GYRO_RIBBLE_THRESH))
        print("  GYRO_LARGE: {}dps".format(self.detector.GYRO_LARGE_THRESH))
        print("  ACC_LARGE: {}g".format(self.detector.LARGE_THRESH))
        print("  MIN_AMPLITUDE: {}g".format(self.detector.MIN_AMPLITUDE))
        print("  WINDOW_SIZE: {} samples".format(self.detector.WINDOW_SIZE))
    
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
        
        # Based on the log analysis, the IMU handler returns values around 1.0g for Z-axis at rest
        # This suggests it includes gravity, so we subtract 1.0g to get motion-only values
        return {
            'ax': accel_data['x'],
            'ay': accel_data['y'],
            'az': accel_data['z'] - 1.0,  # Remove gravity offset
            'gx': gyro_data['x'],
            'gy': gyro_data['y'],
            'gz': gyro_data['z']
        }
    
    def run_detection_loop(self, max_duration_seconds=10, update_rate_ms=10):
        """Run the sine detection loop"""
        print("Starting sine detection loop...")
        print("Looking for sine wave pattern in Z-axis acceleration")
        print("Expected sequence: IDLE -> MOTOR_ON -> FIRST_RISE -> FIRST_FALL -> SECOND_FALL -> SECOND_RISE -> STEADY")
        
        start_time = utime.time()
        last_debug_time = 0
        
        try:
            while True:
                current_time = utime.time()
                
                # Get IMU sample
                sample = self.get_imu_sample()
                
                # Process sample
                state = self.detector.process_sample(sample)
                
                # Print debug info every 0.1 seconds OR every 5 samples (whichever comes first)
                time_since_last = current_time - last_debug_time
                sample_based_debug = self.detector.sample_count % 5 == 0  # Every 5 samples
                
                if time_since_last >= 0.1 or sample_based_debug:
                    print("[{}] Current state: {} | AZ={:.3f} AX={:.3f} AY={:.3f} | GX={:.1f} GY={:.1f} GZ={:.1f} | Time: {:.3f}s".format(
                        self.detector.sample_count,
                        self.detector.get_state_name(),
                        sample['az'], sample['ax'], sample['ay'],
                        sample['gx'], sample['gy'], sample['gz'],
                        time_since_last
                    ))
                    last_debug_time = current_time
                
                # Check for takeoff detection
                if self.detector.is_takeoff_detected():
                    print("SUCCESS: TAKEOFF DETECTED! Reached STEADY state")
                    print("Total samples processed: {}".format(self.detector.sample_count))
                    print("State changes: {}".format(self.detector.state_change_count))
                    print("Detection time: {:.2f} seconds".format(current_time - start_time))
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
        
        print("\nStarting detection mode...")
        detection_system.run_detection_loop(max_duration_seconds=120, update_rate_ms=50)
            
    except Exception as e:
        print("ERROR: {}".format(e))

