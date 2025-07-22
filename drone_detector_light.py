"""
Lightweight Drone Takeoff Detection System
Works with QuecPython and existing IMU Handler

AGGRESSIVE RESET CONCEPT:
- Step 1→2: Movement detected → enter ripples state
- Step 2: Stay in ripples, IMMEDIATE RESET if:
  * X/Y movement > 0.8g (manual handling)
  * Ripples completely lost (motors stopped)
  * Extreme rotation/magnetic interference
- Step 3: Z takeoff → Z return, IMMEDIATE RESET if:
  * X/Y movement > 1.0g (manual handling during takeoff)
  * Motors stopped, excessive rotation, magnetic interference
- Step 4: Stability validation with enhanced requirements

NO TOLERANCE for clearly invalid conditions - instant reset for manual handling detection
"""

import utime
import math
from usr.imu_handler import IMUHandler


class SimpleDeque:
    """Simple deque replacement for QuecPython compatibility"""
    def __init__(self, maxlen=None):
        self.maxlen = maxlen
        self.data = []
    
    def append(self, item):
        """Add item to the right end"""
        self.data.append(item)
        if self.maxlen and len(self.data) > self.maxlen:
            self.data.pop(0)  # Remove from left end
    
    def __len__(self):
        """Return length of deque"""
        return len(self.data)
    
    def __getitem__(self, index):
        """Support indexing like data[-1] or data[-10:]"""
        return self.data[index]
    
    def __iter__(self):
        """Support iteration"""
        return iter(self.data)


# Create deque function for compatibility
def deque(maxlen=None):
    """Factory function to create SimpleDeque with maxlen"""
    return SimpleDeque(maxlen=maxlen)

# ====== DETECTION THRESHOLDS - ADJUST FOR YOUR DRONE ======
# Accelerometer Detection Method
USE_INDIVIDUAL_AXIS = True           # True=direct axis values, False=variance method
INDIVIDUAL_ACC_THRESHOLD = 0.3       # Individual axis acceleration threshold (g) - MUCH HIGHER for drone motors
INDIVIDUAL_GYRO_THRESHOLD = 20.0     # Individual axis gyroscope threshold (rad/s) - RELAXED for drone
INDIVIDUAL_MAG_THRESHOLD = 5.0       # Individual axis magnetometer threshold (μT)

# Variance Method Thresholds (if USE_INDIVIDUAL_AXIS = False)
RIPPLE_THRESHOLD = 0.005             # Variance threshold for motor vibrations - HIGHER
MOVEMENT_VARIANCE_THRESHOLD = 0.01   # Initial movement detection threshold - HIGHER

# Takeoff Sequence Thresholds (Z-axis values are gravity-corrected, 0g = at rest)
Z_HIGH_THRESHOLD = 0.5               # Z-axis takeoff detection (g above rest) - MUCH HIGHER 
Z_RETURN_THRESHOLD = 0.3             # Z-axis return detection (g below rest) - HIGHER
Z_STABLE_MAX = 0.15                  # Maximum Z for stability (g from rest) - TIGHTER

# Immediate Reset Thresholds (no tolerance, instant reset for invalid conditions)
MAX_XY_MOVEMENT_STEP2 = 0.8          # Max X/Y movement in Step 2 before immediate reset
MAX_XY_MOVEMENT_STEP3 = 1.0          # Max X/Y movement in Step 3 before immediate reset  
RIPPLE_LOST_THRESHOLD = 0.1          # Below this = ripples completely lost (motors stopped)

# Enhanced 9D Validation Thresholds - MULTI-TIER SYSTEM:
# STRICT (Step 1, 4): Base thresholds for precision detection
# RELAXED (Step 2, 3): 1.5x base for motor-induced vibrations  
# EXTREME (Step 2 only): 3.0x base for immediate manual handling detection
GYRO_STATIONARY_THRESHOLD = 30      # Base gyroscope threshold (rad/s)
MAG_STABILITY_THRESHOLD = 1.0        # Base magnetometer stability threshold

# Detection Parameters
ANALYSIS_WINDOW = 20                 # Samples for analysis window
BUFFER_SIZE = 100                    # Data buffer size  
MIN_STABLE_SAMPLES = 20              # Minimum stable samples for completion - DOUBLED
UPDATE_RATE_MS = 100                 # Update rate in milliseconds



# Debug Parameters
DEBUG_MODE = True                    # Enable detailed debug output
DEBUG_SENSOR_VALUES = True           # Show raw sensor values
DEBUG_CALCULATIONS = True            # Show threshold calculations
DEBUG_EVERY_N_SAMPLES = 2            # Debug output frequency


class LightDroneDetector:
    """Lightweight drone takeoff detection using enhanced 9D IMU validation"""
    
    def __init__(self, config_manager):
        # Initialize IMU handler
        self.imu_handler = IMUHandler(config_manager)
        
        # Data buffers for analysis
        self.data_buffer = {
            'acc_x': deque(maxlen=BUFFER_SIZE),
            'acc_y': deque(maxlen=BUFFER_SIZE), 
            'acc_z': deque(maxlen=BUFFER_SIZE),
            'gyro_x': deque(maxlen=BUFFER_SIZE),
            'gyro_y': deque(maxlen=BUFFER_SIZE),
            'gyro_z': deque(maxlen=BUFFER_SIZE),
            'mag_x': deque(maxlen=BUFFER_SIZE),
            'mag_y': deque(maxlen=BUFFER_SIZE),
            'mag_z': deque(maxlen=BUFFER_SIZE),
            'timestamp': deque(maxlen=BUFFER_SIZE)
        }
        
        # Detection state
        self.sequence_step = 1               # 1=waiting, 2=ripples, 3=takeoff, 4=stabilizing
        self.takeoff_state = 'waiting'       # waiting, ripples, takeoff, stabilized
        self.sample_count = 0
        self.step_start_sample = 0
        self.stable_count = 0
        self.reset_count = 0
        self.z_high_detected = False
        

        
        # Detection results
        self.takeoff_detected = False
        self.detection_complete = False
        self.last_detection_time = 0
        
        # Status callback
        self.status_callback = None
        
        print("Lightweight Drone Detector initialized")
        detection_method = 'INDIVIDUAL AXIS' if USE_INDIVIDUAL_AXIS else 'VARIANCE'
        print("Method: {}".format(detection_method))
        print("Thresholds: ACC={}g, GYRO={}rad/s, MAG={}".format(
            INDIVIDUAL_ACC_THRESHOLD, INDIVIDUAL_GYRO_THRESHOLD, INDIVIDUAL_MAG_THRESHOLD))
        print("IMMEDIATE RESET concept:")
        print("  Step 2: XY > {:.1f}g = RESET | Ripples lost = RESET".format(MAX_XY_MOVEMENT_STEP2))
        print("  Step 3: XY > {:.1f}g = RESET | Motors stop = RESET".format(MAX_XY_MOVEMENT_STEP3))
        print("  Z thresholds: HIGH={:.1f}g, RETURN={:.1f}g, STABLE={:.1f}g".format(
            Z_HIGH_THRESHOLD, Z_RETURN_THRESHOLD, Z_STABLE_MAX))
        print("NOTE: Z-axis acceleration is gravity-corrected (1g subtracted from raw readings)")
    
    def set_status_callback(self, callback):
        """Set callback function for status updates"""
        self.status_callback = callback
    
    def _log_status(self, message, step_info=None):
        """Log status message and call callback if set"""
        timestamp = utime.time()
        status_data = {
            'timestamp': timestamp,
            'message': message,
            'step': self.sequence_step,
            'state': self.takeoff_state,
            'sample_count': self.sample_count,
            'reset_count': self.reset_count
        }
        if step_info:
            status_data.update(step_info)
            
        print("[{}] {}".format(timestamp, message))
        
        if self.status_callback:
            self.status_callback(status_data)
    
    def start(self):
        """Start IMU handler and detection"""
        if not self.imu_handler.start():
            print("ERROR: Failed to start IMU handler")
            return False
            
        print("SUCCESS: IMU handler started")
        self._log_status("Detection system started")
        return True
    
    def stop(self):
        """Stop detection and IMU handler"""
        self.imu_handler.stop()
        self._log_status("Detection system stopped")
        print("STOP: Detection system stopped")
    
    def reset_detection(self, reason="Manual reset"):
        """Reset detection sequence to start over"""
        self.sequence_step = 1
        self.takeoff_state = 'waiting'
        self.step_start_sample = self.sample_count
        self.stable_count = 0
        self.z_high_detected = False
        self.takeoff_detected = False
        self.detection_complete = False
        self.reset_count += 1
        
        self._log_status("RESET: {}".format(reason))
    
    def get_detection_status(self):
        """Get current detection status"""
        return {
            'step': self.sequence_step,
            'state': self.takeoff_state,
            'takeoff_detected': self.takeoff_detected,
            'detection_complete': self.detection_complete,
            'sample_count': self.sample_count,
            'reset_count': self.reset_count,
            'buffer_size': len(self.data_buffer['acc_x'])
        }
    
    def update(self):
        """Single detection update cycle - call this regularly"""
        # Get fresh IMU data
        accel_data = self.imu_handler.get_accel()
        gyro_data = self.imu_handler.get_gyro() 
        mag_data = self.imu_handler.get_mag()
        
        # Add to buffers (subtract 1g from Z-axis to remove gravity baseline)
        current_time = utime.time()
        self.data_buffer['acc_x'].append(accel_data['x'])
        self.data_buffer['acc_y'].append(accel_data['y'])
        self.data_buffer['acc_z'].append(accel_data['z'] - 1.0)  # Remove 1g gravity baseline
        self.data_buffer['gyro_x'].append(gyro_data['x'])
        self.data_buffer['gyro_y'].append(gyro_data['y'])
        self.data_buffer['gyro_z'].append(gyro_data['z'])
        self.data_buffer['mag_x'].append(mag_data['x'])
        self.data_buffer['mag_y'].append(mag_data['y'])
        self.data_buffer['mag_z'].append(mag_data['z'])
        self.data_buffer['timestamp'].append(current_time)
        
        self.sample_count += 1
        
        # Need minimum samples for analysis
        if len(self.data_buffer['acc_x']) < ANALYSIS_WINDOW:
            return
            
        # Perform enhanced 9D detection
        self._detect_takeoff_enhanced_9d()
    
    def _detect_takeoff_enhanced_9d(self):
        """Enhanced 9D takeoff detection with gyroscope and magnetometer validation"""
        # Convert buffers to lists for analysis
        acc_x = list(self.data_buffer['acc_x'])
        acc_y = list(self.data_buffer['acc_y'])
        acc_z = list(self.data_buffer['acc_z'])
        gyro_x = list(self.data_buffer['gyro_x'])
        gyro_y = list(self.data_buffer['gyro_y'])
        gyro_z = list(self.data_buffer['gyro_z'])
        mag_x = list(self.data_buffer['mag_x'])
        mag_y = list(self.data_buffer['mag_y'])
        mag_z = list(self.data_buffer['mag_z'])
        
        # Debug output for sensor values
        if DEBUG_SENSOR_VALUES and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
            print("=== SENSOR DEBUG [Sample {}] ===".format(self.sample_count))
            print("ACC: X={:.3f} Y={:.3f} Z={:.3f} (Z already gravity-corrected)".format(acc_x[-1], acc_y[-1], acc_z[-1]))
            print("GYRO: X={:.3f} Y={:.3f} Z={:.3f}".format(gyro_x[-1], gyro_y[-1], gyro_z[-1]))
            print("MAG: X={:.3f} Y={:.3f} Z={:.3f}".format(mag_x[-1], mag_y[-1], mag_z[-1]))
        
        # === HELPER FUNCTIONS ===
        def calculate_variance(values):
            if len(values) < 2:
                return 0
            mean_val = sum(values) / len(values)
            return sum((x - mean_val)**2 for x in values) / len(values)
        
        # === ENHANCED 9D IMU CALCULATIONS ===
        # Gyroscope magnitude calculation
        recent_gyro_x = gyro_x[-ANALYSIS_WINDOW:]
        recent_gyro_y = gyro_y[-ANALYSIS_WINDOW:]
        recent_gyro_z = gyro_z[-ANALYSIS_WINDOW:]
        gyro_magnitudes = [math.sqrt(gx**2 + gy**2 + gz**2) for gx, gy, gz in 
                          zip(recent_gyro_x, recent_gyro_y, recent_gyro_z)]
        mean_gyro_magnitude = sum(gyro_magnitudes) / len(gyro_magnitudes)
        current_gyro_magnitude = math.sqrt(gyro_x[-1]**2 + gyro_y[-1]**2 + gyro_z[-1]**2)
        
        # Magnetometer stability calculation
        recent_mag_x = mag_x[-ANALYSIS_WINDOW:]
        recent_mag_y = mag_y[-ANALYSIS_WINDOW:]
        recent_mag_z = mag_z[-ANALYSIS_WINDOW:]
        
        def calculate_std(values):
            if len(values) == 0:
                return 0
            mean_val = sum(values) / len(values)
            variance = sum((x - mean_val)**2 for x in values) / len(values)
            return math.sqrt(variance)
            
        mag_x_std = calculate_std(recent_mag_x)
        mag_y_std = calculate_std(recent_mag_y)
        mag_z_std = calculate_std(recent_mag_z)
        max_mag_std = max(mag_x_std, mag_y_std, mag_z_std)
        
        # Debug output for calculations
        if DEBUG_CALCULATIONS and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
            print("=== CALCULATION DEBUG ===")
            print("Gyro magnitude: {:.3f} (thresholds: STRICT={:.3f}, RELAXED={:.3f}, EXTREME={:.3f})".format(
                mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD, GYRO_STATIONARY_THRESHOLD*1.5, GYRO_STATIONARY_THRESHOLD*3.0))
            print("Current gyro mag: {:.3f}".format(current_gyro_magnitude))
            print("Mag stability: {:.3f} (thresholds: STRICT={:.3f}, RELAXED={:.3f}, EXTREME={:.3f})".format(
                max_mag_std, MAG_STABILITY_THRESHOLD, MAG_STABILITY_THRESHOLD*1.5, MAG_STABILITY_THRESHOLD*3.0))
            print("Mag std - X:{:.3f} Y:{:.3f} Z:{:.3f}".format(mag_x_std, mag_y_std, mag_z_std))
        
        # === DETECTION METHOD SELECTION ===
        if USE_INDIVIDUAL_AXIS:
            # Individual axis approach
            acc_movement = (abs(acc_x[-1]) > INDIVIDUAL_ACC_THRESHOLD or 
                           abs(acc_y[-1]) > INDIVIDUAL_ACC_THRESHOLD or 
                           abs(acc_z[-1]) > INDIVIDUAL_ACC_THRESHOLD)
            motor_running = acc_movement
            z_takeoff_detected = acc_z[-1] > Z_HIGH_THRESHOLD
            z_stabilized = abs(acc_z[-1]) < Z_STABLE_MAX
            
        else:
            # Variance approach
            def calculate_variance(values):
                if len(values) < 2:
                    return 0
                mean_val = sum(values) / len(values)
                return sum((x - mean_val)**2 for x in values) / len(values)
            
            accel_variance = (calculate_variance(acc_x[-10:]) + 
                             calculate_variance(acc_y[-10:]) + 
                             calculate_variance(acc_z[-10:]))
            recent_var_z = calculate_variance(acc_z[-ANALYSIS_WINDOW:])
            
            acc_movement = accel_variance > MOVEMENT_VARIANCE_THRESHOLD
            motor_running = recent_var_z > RIPPLE_THRESHOLD
            z_takeoff_detected = sum(acc_z[-ANALYSIS_WINDOW:]) / ANALYSIS_WINDOW > Z_HIGH_THRESHOLD
            z_stabilized = sum(abs(z) for z in acc_z[-ANALYSIS_WINDOW:]) / ANALYSIS_WINDOW < Z_STABLE_MAX
        
        # Debug output for detection conditions
        if DEBUG_MODE and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
            print("=== DETECTION CONDITIONS DEBUG ===")
            print("Step: {} | State: {}".format(self.sequence_step, self.takeoff_state))
            print("ACC movement: {} | Motor running: {}".format(acc_movement, motor_running))
            print("Z takeoff detected: {} | Z stabilized: {}".format(z_takeoff_detected, z_stabilized))
            if USE_INDIVIDUAL_AXIS:
                print("Individual axis method - ACC thresholds: X={:.3f}>{:.3f}, Y={:.3f}>{:.3f}, Z={:.3f}>{:.3f}".format(
                    abs(acc_x[-1]), INDIVIDUAL_ACC_THRESHOLD,
                    abs(acc_y[-1]), INDIVIDUAL_ACC_THRESHOLD, 
                    abs(acc_z[-1]), INDIVIDUAL_ACC_THRESHOLD))
            else:
                accel_variance = (calculate_variance(acc_x[-10:]) + 
                                 calculate_variance(acc_y[-10:]) + 
                                 calculate_variance(acc_z[-10:]))
                print("Variance method - Accel variance: {:.6f} (thresh: {:.6f})".format(
                    accel_variance, MOVEMENT_VARIANCE_THRESHOLD))
        
        current_sample = self.sample_count
        
        # === ENHANCED 9D SEQUENCE VALIDATION ===
        if self.sequence_step == 1:
            # Step 1: Wait for movement with enhanced 9D validation
            gyro_stationary = mean_gyro_magnitude < GYRO_STATIONARY_THRESHOLD
            mag_stable = max_mag_std < MAG_STABILITY_THRESHOLD
            
            # Debug step 1 validation
            if DEBUG_MODE and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
                print("=== STEP 1 VALIDATION ===")
                print("ACC movement: {} | Gyro stationary: {} | Mag stable: {}".format(
                    acc_movement, gyro_stationary, mag_stable))
                print("Gyro: {:.3f} < {:.3f} = {}".format(
                    mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD, gyro_stationary))
                print("Mag: {:.3f} < {:.3f} = {}".format(
                    max_mag_std, MAG_STABILITY_THRESHOLD, mag_stable))
            
            if acc_movement and gyro_stationary and mag_stable:
                # Valid movement detected - advance to step 2
                self.sequence_step = 2
                self.step_start_sample = current_sample
                self.takeoff_state = 'ripples'
                
                step_info = {
                    'gyro_magnitude': mean_gyro_magnitude,
                    'mag_stability': max_mag_std,
                    'method': 'individual_axis' if USE_INDIVIDUAL_AXIS else 'variance'
                }
                self._log_status("STEP 2: Enhanced movement detected", step_info)
                
            elif acc_movement and not gyro_stationary:
                msg = "BLOCKED: Manual handling detected (gyro_mag={:.3f} > {})".format(
                    mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD)
                self._log_status(msg)
            elif acc_movement and not mag_stable:
                msg = "BLOCKED: Magnetic interference detected (mag_std={:.3f} > {})".format(
                    max_mag_std, MAG_STABILITY_THRESHOLD)
                self._log_status(msg)
        
        elif self.sequence_step == 2:
            # Step 2: Stay in ripples, wait for Z takeoff, IMMEDIATE RESET for big X/Y movements
            
            # === IMMEDIATE RESET CONDITIONS (no tolerance) ===
            # Check for excessive X/Y movement (manual handling)
            max_xy_movement = max(abs(acc_x[-1]), abs(acc_y[-1]))
            if max_xy_movement > MAX_XY_MOVEMENT_STEP2:
                msg = "Reset: Excessive X/Y movement in step 2 (max={:.3f} > {:.3f}) - manual handling detected".format(
                    max_xy_movement, MAX_XY_MOVEMENT_STEP2)
                self.reset_detection(msg)
                return
            
            # Check if ripples completely lost
            if USE_INDIVIDUAL_AXIS:
                current_movement = max(abs(acc_x[-1]), abs(acc_y[-1]), abs(acc_z[-1]))
                ripples_lost = current_movement < RIPPLE_LOST_THRESHOLD
            else:
                recent_var_z = calculate_variance(acc_z[-ANALYSIS_WINDOW:])
                ripples_lost = recent_var_z < (RIPPLE_THRESHOLD / 10)  # Much lower threshold
            
            if ripples_lost:
                msg = "Reset: Ripples completely lost in step 2 - motors stopped"
                self.reset_detection(msg)
                return
            
            # Check for extreme rotation/magnetic disturbance (immediate reset)
            if mean_gyro_magnitude > (GYRO_STATIONARY_THRESHOLD * 3.0):
                msg = "Reset: Extreme rotation in step 2 (gyro_mag={:.3f}) - manual handling".format(mean_gyro_magnitude)
                self.reset_detection(msg)
                return
                
            if max_mag_std > (MAG_STABILITY_THRESHOLD * 3.0):
                msg = "Reset: Extreme magnetic disturbance in step 2 (mag_std={:.3f}) - interference".format(max_mag_std)
                self.reset_detection(msg)
                return
            
            # === NORMAL OPERATION (ripples maintained) ===
            # Debug step 2 validation
            if DEBUG_MODE and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
                print("=== STEP 2 VALIDATION ===")
                print("XY movement: {:.3f} < {:.3f} | Ripples present: {}".format(
                    max_xy_movement, MAX_XY_MOVEMENT_STEP2, not ripples_lost))
                print("Gyro: {:.3f} | Mag: {:.3f} | Z takeoff: {}".format(
                    mean_gyro_magnitude, max_mag_std, z_takeoff_detected))
            
            # Check if Z-axis goes POSITIVE (upward) for takeoff
            if z_takeoff_detected:
                self.sequence_step = 3
                self.step_start_sample = current_sample
                self.z_high_detected = True
                self.takeoff_state = 'takeoff'
                
                z_val = acc_z[-1] if USE_INDIVIDUAL_AXIS else sum(acc_z[-ANALYSIS_WINDOW:]) / ANALYSIS_WINDOW
                step_info = {
                    'z_value': z_val,
                    'xy_movement': max_xy_movement,
                    'gyro_magnitude': mean_gyro_magnitude,
                    'mag_stability': max_mag_std
                }
                self._log_status("STEP 3: Takeoff detected (ripples maintained)", step_info)
            # Otherwise, stay in step 2 with ripples maintained
        
        elif self.sequence_step == 3:
            # Step 3: Z high -> Z return, IMMEDIATE RESET for any unexpected conditions
            
            # === IMMEDIATE RESET CONDITIONS (no tolerance) ===
            # Check for excessive X/Y movement (manual handling during takeoff)
            max_xy_movement = max(abs(acc_x[-1]), abs(acc_y[-1]))
            if max_xy_movement > MAX_XY_MOVEMENT_STEP3:
                msg = "Reset: Excessive X/Y movement in step 3 (max={:.3f} > {:.3f}) - manual handling during takeoff".format(
                    max_xy_movement, MAX_XY_MOVEMENT_STEP3)
                self.reset_detection(msg)
                return
            
            # Check for excessive rotation (immediate reset)
            if mean_gyro_magnitude > (GYRO_STATIONARY_THRESHOLD * 2.0):
                msg = "Reset: Excessive rotation in step 3 (gyro_mag={:.3f} > {:.3f}) - manual handling".format(
                    mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD * 2.0)
                self.reset_detection(msg)
                return
                
            # Check for magnetic disturbance (immediate reset)
            if max_mag_std > (MAG_STABILITY_THRESHOLD * 2.0):
                msg = "Reset: Magnetic disturbance in step 3 (mag_std={:.3f} > {:.3f}) - interference".format(
                    max_mag_std, MAG_STABILITY_THRESHOLD * 2.0)
                self.reset_detection(msg)
                return
            
            # Check if motors stopped (immediate reset)
            if USE_INDIVIDUAL_AXIS:
                current_movement = max(abs(acc_x[-1]), abs(acc_y[-1]), abs(acc_z[-1]))
                motors_stopped = current_movement < RIPPLE_LOST_THRESHOLD
            else:
                recent_var_z = calculate_variance(acc_z[-ANALYSIS_WINDOW:])
                motors_stopped = recent_var_z < (RIPPLE_THRESHOLD / 10)
                
            if motors_stopped:
                msg = "Reset: Motors stopped in step 3 - lost all movement"
                self.reset_detection(msg)
                return
            
            # Debug step 3 validation
            if DEBUG_MODE and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
                print("=== STEP 3 VALIDATION ===")
                print("Z high detected: {} | XY movement: {:.3f} < {:.3f}".format(
                    self.z_high_detected, max_xy_movement, MAX_XY_MOVEMENT_STEP3))
                print("Gyro: {:.3f} | Mag: {:.3f} | Motors running: {}".format(
                    mean_gyro_magnitude, max_mag_std, not motors_stopped))
            
            # === NORMAL OPERATION ===
            if self.z_high_detected:
                # Z was positive, now check if it goes NEGATIVE and returns to ripples/movement
                if USE_INDIVIDUAL_AXIS:
                    z_returned = acc_z[-1] < -Z_RETURN_THRESHOLD
                else:
                    z_mean = sum(acc_z[-ANALYSIS_WINDOW:]) / ANALYSIS_WINDOW
                    z_returned = z_mean < -Z_RETURN_THRESHOLD
                
                if z_returned:
                    # Z returned to negative - advance to step 4
                    self.sequence_step = 4
                    self.step_start_sample = current_sample
                    self.stable_count = 0
                    self.takeoff_state = 'stabilizing'
                    
                    z_val = acc_z[-1] if USE_INDIVIDUAL_AXIS else z_mean
                    step_info = {
                        'z_value': z_val,
                        'xy_movement': max_xy_movement,
                        'gyro_magnitude': mean_gyro_magnitude,
                        'mag_stability': max_mag_std
                    }
                    self._log_status("STEP 4: Stabilizing (Z returned)", step_info)
                # Otherwise, wait for Z to return negative (all conditions already validated above)
            else:
                # Should not reach here, but safety check
                self.reset_detection("Step 3 logic error")
        
        elif self.sequence_step == 4:
            # Step 4: Must be STABLE in ripples + ENHANCED 9D validation (STRICT)
            # Strict thresholds - confirm stable hover with minimal rotation and magnetic stability
            gyro_stable = mean_gyro_magnitude < GYRO_STATIONARY_THRESHOLD
            mag_stable = max_mag_std < MAG_STABILITY_THRESHOLD
            
            if USE_INDIVIDUAL_AXIS:
                movement_present = acc_movement
                z_stable = z_stabilized  # abs(acc_z[-1]) < Z_STABLE_MAX
            else:
                recent_var_z = calculate_variance(acc_z[-ANALYSIS_WINDOW:])
                movement_present = recent_var_z > RIPPLE_THRESHOLD
                recent_z_abs = [abs(z) for z in acc_z[-ANALYSIS_WINDOW:]]
                z_stable = sum(recent_z_abs) / len(recent_z_abs) < Z_STABLE_MAX
            
            # Debug step 4 validation
            if DEBUG_MODE and self.sample_count % DEBUG_EVERY_N_SAMPLES == 0:
                print("=== STEP 4 VALIDATION ===")
                print("Motor: {} | Z stable: {} | Gyro stable: {} | Mag stable: {}".format(
                    movement_present, z_stable, gyro_stable, mag_stable))
                print("Stable count: {}".format(self.stable_count))
                print("Gyro: {:.3f} < {:.3f} = {}".format(
                    mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD, gyro_stable))
                print("Mag: {:.3f} < {:.3f} = {}".format(
                    max_mag_std, MAG_STABILITY_THRESHOLD, mag_stable))
            
            if movement_present and z_stable and gyro_stable and mag_stable:
                # Count consecutive stable samples with all sensors confirming stability
                self.stable_count += 1
                
                # Check if we've had enough consecutive stable samples (minimum confirmation)
                if self.stable_count >= MIN_STABLE_SAMPLES:
                    # SUCCESSFUL TAKEOFF SEQUENCE COMPLETED!
                    if self.takeoff_state != 'stabilized':  # Only validate once
                        self.takeoff_detected = True
                        self.detection_complete = True
                        self.takeoff_state = 'stabilized'
                        self.last_detection_time = utime.time()
                        
                        step_info = {
                            'stable_count': self.stable_count,
                            'total_samples': self.sample_count,
                            'reset_count': self.reset_count,
                            'gyro_magnitude': mean_gyro_magnitude,
                            'mag_stability': max_mag_std
                        }
                        self._log_status("SUCCESS: ENHANCED TAKEOFF SEQUENCE COMPLETED!", step_info)
                        
                        self.sequence_step = 5  # Mark as completed to stop further processing
            elif not z_stable:
                # Big Z movement again - reset stability counter but don't fail yet
                self.stable_count = 0
                if USE_INDIVIDUAL_AXIS:
                    msg = "Stability reset: Z movement detected (|ACC_Z|={:.3f} > {:.3f})".format(
                        abs(acc_z[-1]), Z_STABLE_MAX)
                else:
                    z_abs_mean = sum(recent_z_abs) / len(recent_z_abs)
                    msg = "Stability reset: Z movement detected (|Z|={:.3f} > {:.3f})".format(
                        z_abs_mean, Z_STABLE_MAX)
                self._log_status(msg)
            elif not movement_present:
                # Lost movement/ripples in stabilization - INVALID
                if USE_INDIVIDUAL_AXIS:
                    msg = "Reset: Lost movement in step 4 (no significant axis values)"
                else:
                    msg = "Reset: Lost ripples in step 4 (variance={:.6f} < {:.6f})".format(
                        recent_var_z, RIPPLE_THRESHOLD)
                self.reset_detection(msg)
            elif not gyro_stable:
                # Excessive rotation during stabilization - likely manual handling
                msg = "Reset: Rotation detected during stabilization (gyro_mag={:.3f} > {:.3f}) - manual handling".format(
                    mean_gyro_magnitude, GYRO_STATIONARY_THRESHOLD)
                self.reset_detection(msg)
            elif not mag_stable:
                # Magnetic instability during stabilization
                msg = "Reset: Magnetic instability during stabilization (mag_std={:.3f} > {:.3f}) - interference".format(
                    max_mag_std, MAG_STABILITY_THRESHOLD)
                self.reset_detection(msg)
        
        elif self.sequence_step == 5:
            # Sequence completed - no further processing needed
            pass
    
    def run_detection_loop(self, max_duration_seconds=300):
        """Run continuous detection loop"""
        print("Starting continuous detection loop...")
        start_time = utime.time()
        
        try:
            while True:
                # Update detection
                self.update()
                
                # Check if detection completed
                if self.detection_complete:
                    print("SUCCESS: Takeoff detected successfully after {} samples!".format(self.sample_count))
                    break
                
                # Check timeout
                if utime.time() - start_time > max_duration_seconds:
                    print("TIMEOUT: Detection timeout after {} seconds".format(max_duration_seconds))
                    break
                
                # Sleep for update rate
                utime.sleep_ms(UPDATE_RATE_MS)
                
        except KeyboardInterrupt:
            print("STOP: Detection stopped by user")
        except Exception as e:
            print("ERROR: Error in detection loop: {}".format(e))
        finally:
            self.stop()
    
    def run_calibration_mode(self, duration_seconds=30):
        """Run calibration mode to observe sensor values and help set thresholds"""
        print("=== CALIBRATION MODE ===")
        print("Starting {} second calibration session...".format(duration_seconds))
        print("This will show continuous sensor readings to help you understand normal vs drone values")
        print("Instructions:")
        print("1. First, keep the sensor completely still")
        print("2. Then, simulate normal drone startup (props spinning)")
        print("3. Finally, try manual movement to see high values")
        print("")
        
        start_time = utime.time()
        sample_count = 0
        
        try:
            while utime.time() - start_time < duration_seconds:
                # Get fresh IMU data
                accel_data = self.imu_handler.get_accel()
                gyro_data = self.imu_handler.get_gyro() 
                mag_data = self.imu_handler.get_mag()
                
                # Add to buffers (subtract 1g from Z-axis to remove gravity baseline)
                current_time = utime.time()
                self.data_buffer['acc_x'].append(accel_data['x'])
                self.data_buffer['acc_y'].append(accel_data['y'])
                self.data_buffer['acc_z'].append(accel_data['z'] - 1.0)  # Remove 1g gravity baseline
                self.data_buffer['gyro_x'].append(gyro_data['x'])
                self.data_buffer['gyro_y'].append(gyro_data['y'])
                self.data_buffer['gyro_z'].append(gyro_data['z'])
                self.data_buffer['mag_x'].append(mag_data['x'])
                self.data_buffer['mag_y'].append(mag_data['y'])
                self.data_buffer['mag_z'].append(mag_data['z'])
                self.data_buffer['timestamp'].append(current_time)
                
                sample_count += 1
                
                # Show detailed readings every few samples
                if sample_count % 10 == 0:
                    elapsed = int(utime.time() - start_time)
                    gyro_mag = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)
                    acc_mag = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
                    
                    # Calculate gravity-corrected values for display
                    acc_z_corrected = accel_data['z'] - 1.0
                    acc_mag_corrected = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + acc_z_corrected**2)
                    
                    print("[{}s] ACC: {:.3f},{:.3f},{:.3f} (corrected Z={:.3f}, mag={:.3f}) | GYRO: {:.3f},{:.3f},{:.3f} (mag={:.3f}) | MAG: {:.3f},{:.3f},{:.3f}".format(
                        elapsed,
                        accel_data['x'], accel_data['y'], accel_data['z'], acc_z_corrected, acc_mag_corrected,
                        gyro_data['x'], gyro_data['y'], gyro_data['z'], gyro_mag,
                        mag_data['x'], mag_data['y'], mag_data['z']
                    ))
                    
                    # Show threshold analysis if we have enough data
                    if len(self.data_buffer['acc_x']) >= ANALYSIS_WINDOW:
                        recent_gyro_x = list(self.data_buffer['gyro_x'])[-ANALYSIS_WINDOW:]
                        recent_gyro_y = list(self.data_buffer['gyro_y'])[-ANALYSIS_WINDOW:]
                        recent_gyro_z = list(self.data_buffer['gyro_z'])[-ANALYSIS_WINDOW:]
                        gyro_magnitudes = [math.sqrt(gx**2 + gy**2 + gz**2) for gx, gy, gz in 
                                          zip(recent_gyro_x, recent_gyro_y, recent_gyro_z)]
                        mean_gyro_magnitude = sum(gyro_magnitudes) / len(gyro_magnitudes)
                        
                        print("    -> Mean gyro mag: {:.3f} | Current thresholds: STATIONARY={} RELAXED={} MODERATE={}".format(
                            mean_gyro_magnitude, 
                            GYRO_STATIONARY_THRESHOLD, 
                            GYRO_STATIONARY_THRESHOLD * 1.5,
                            GYRO_STATIONARY_THRESHOLD * 2.0
                        ))
                
                utime.sleep_ms(UPDATE_RATE_MS)
                
        except KeyboardInterrupt:
            print("STOP: Calibration stopped by user")
        except Exception as e:
            print("ERROR: Error in calibration: {}".format(e))
        
        print("=== CALIBRATION COMPLETE ===")
        print("Use the observed values to adjust thresholds in the code")
        print("Important notes:")
        print("- Z-axis values are gravity-corrected (0g = at rest)")
        print("- When still: Z should be ~0g, X&Y should be small")
        print("- During drone startup: expect Z movement and gyro activity")
        print("- GYRO_STATIONARY_THRESHOLD: Set above normal 'still' values")
        print("- Individual thresholds: Set above normal vibration levels")
        print("- Consider drone prop wash and vibration levels")


def create_simple_detector(config_manager):
    """Create and return a simple detector instance"""
    return LightDroneDetector(config_manager)


# Example usage
if __name__ == "__main__":
    try:
        # This would need to be imported properly in QuecPython
        from usr.config_manager import ConfigManager
        
        # Create config manager
        config_mgr = ConfigManager()
        
        # Create detector
        detector = LightDroneDetector(config_mgr)
        
        # Set up status callback (optional)
        def status_callback(status_data):
            print("Status: {}".format(status_data['message']))
        
        detector.set_status_callback(status_callback)
        
        # Start detection
        if detector.start():
            print("=== CHOOSE MODE ===")
            print("For calibration and threshold tuning, uncomment the calibration line below")
            print("For normal detection, the detection loop will run")
            
            # CALIBRATION MODE - Uncomment to run calibration first
            # detector.run_calibration_mode(duration_seconds=60)
            
            # DETECTION MODE - Normal operation
            detector.run_detection_loop(max_duration_seconds=60)
        else:
            print("ERROR: Failed to start detector")
            
    except Exception as e:
        print("ERROR: {}".format(e)) 