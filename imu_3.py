import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy.signal import find_peaks
import serial
import time
from collections import deque
import threading
from matplotlib.widgets import Button

# Configuration parameters - ADJUST AS NEEDED
SERIAL_PORT = 'COM12'                 # Serial port for live data
BAUD_RATE = 115200                   # Baud rate for serial communication
#CSV_FILE = 'data_with_ripples.csv'  # Backup data file for testing
CSV_FILE = 'imu_data_20250718_100137.csv'  # Backup data file for testing
ACC_COLS = ['acc_x', 'acc_y', 'acc_z']  # Names of accelerometer columns
GYRO_COLS = ['gyro_x', 'gyro_y', 'gyro_z'] # Names of gyroscope columns
MAG_COLS = ['mag_x', 'mag_y', 'mag_z']   # Names of magnetometer columns
RIPPLE_WINDOW = 20                   # Window size for ripple detection
Z_WINDOW = 50                        # Window size for z-change detection
RIPPLE_THRESHOLD = 0.0005             # Lowered variance threshold for ripples
Z_CHANGE_THRESHOLD = 0.5         # Minimum z-change for takeoff detection
BUFFER_SIZE = 200                    # Reduced for better performance
PLOT_WINDOW = 100                    # Reduced for faster drawing
USE_LIVE_DATA = True                 # Set to False to use CSV file instead

# Step 3 Detection Thresholds
Z_HIGH_THRESHOLD = 0.1                # Threshold for detecting Z-axis going high (Step 2->3)
Z_RETURN_THRESHOLD = 0.1             # Threshold for detecting Z-axis returning from high (Step 3->4)
XY_RIPPLE_MAX = 0.1                   # Maximum allowed X/Y values during takeoff (should stay in ripple range)
Z_STABLE_MAX = 0.2                    # Maximum Z-axis absolute value allowed in Step 4 for stability

# Gyroscope and Magnetometer Thresholds (New)
GYRO_STATIONARY_THRESHOLD = 0.1      # Increased from 0.1 to allow for some motor-induced vibration
MAG_STABILITY_THRESHOLD = 1.0        # Increased from 0.5 to allow for some motor-induced vibration

# ALTERNATIVE: Individual Axis Thresholds (instead of variance)
USE_INDIVIDUAL_AXIS = True           # Set to True to use individual axis values instead of variance
INDIVIDUAL_ACC_THRESHOLD = 0.05      # Individual axis acceleration threshold (g)
INDIVIDUAL_GYRO_THRESHOLD = 20.0      # Individual axis gyroscope threshold (rad/s)
INDIVIDUAL_MAG_THRESHOLD = 5.0       # Individual axis magnetometer threshold (μT)

class RealTimeDroneDetector:
    def __init__(self):
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
        
        # Takeoff detection state with strict sequence validation
        self.takeoff_state = 'waiting'  # waiting, ripples, takeoff, stabilized
        self.transition_idx = -1
        self.takeoff_idx = -1
        self.stabilization_idx = -1
        self.sample_count = 0
        
        # Sequence validation variables (NO STATIC TIMING!)
        self.sequence_step = 1  # 1=zeros, 2=ripples, 3=Z_high_then_ripples, 4=stable_ripples
        self.z_high_detected = False  # Track if Z went high during step 3
        self.min_stable_samples = 10  # Minimum samples to confirm stability (not max time!)
        self.stable_count = 0    # Counter for stable samples
        self.step_start_sample = 0   # When current step started (for reference only)
        self.reset_count = 0     # Track number of resets
        
        # Serial connection
        self.serial_conn = None
        self.running = False
        
        # Setup plot - OPTIMIZED: Only 4 plots instead of 9 for better performance
        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 8))
        self.fig.suptitle("Real-Time Drone Takeoff Detection - FAST MODE", fontsize=16)
        
        # Initialize plot lines
        self.lines = {}
        self.time_data = deque(maxlen=PLOT_WINDOW)
        
        # Detection markers for visualization
        self.detection_markers = {
            'motor_start': [],
            'takeoff': [],
            'stabilization': []
        }
        
        # Plot 1: All Accelerometer data (3 lines on 1 plot)
        self.lines['acc_x'], = self.axes[0].plot([], [], 'r-', label='ACC_X', linewidth=1.5)
        self.lines['acc_y'], = self.axes[0].plot([], [], 'g-', label='ACC_Y', linewidth=1.5)
        self.lines['acc_z'], = self.axes[0].plot([], [], 'b-', label='ACC_Z', linewidth=1.5)
        self.axes[0].set_ylabel("Acceleration (g)")
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].legend(loc='upper right')
        self.axes[0].set_xlim(0, PLOT_WINDOW)
        self.axes[0].set_ylim(-2, 2)
        
        # Plot 2: All Gyroscope data (3 lines on 1 plot)  
        self.lines['gyro_x'], = self.axes[1].plot([], [], 'r--', label='GYRO_X', linewidth=1.5)
        self.lines['gyro_y'], = self.axes[1].plot([], [], 'g--', label='GYRO_Y', linewidth=1.5)
        self.lines['gyro_z'], = self.axes[1].plot([], [], 'b--', label='GYRO_Z', linewidth=1.5)
        self.axes[1].set_ylabel("Angular Velocity (deg/s)")
        self.axes[1].grid(True, alpha=0.3)
        self.axes[1].legend(loc='upper right')
        self.axes[1].set_xlim(0, PLOT_WINDOW)
        self.axes[1].set_ylim(-10, 10)
        
        # Plot 3: All Magnetometer data (3 lines on 1 plot)
        self.lines['mag_x'], = self.axes[2].plot([], [], 'r:', label='MAG_X', linewidth=1.5)
        self.lines['mag_y'], = self.axes[2].plot([], [], 'g:', label='MAG_Y', linewidth=1.5)
        self.lines['mag_z'], = self.axes[2].plot([], [], 'b:', label='MAG_Z', linewidth=1.5)
        self.axes[2].set_ylabel("Magnetic Field (μT)")
        self.axes[2].grid(True, alpha=0.3)
        self.axes[2].legend(loc='upper right')
        self.axes[2].set_xlim(0, PLOT_WINDOW)
        self.axes[2].set_ylim(-100, 100)
        
        # Plot 4: Magnitude plot
        self.lines['magnitude'], = self.axes[3].plot([], [], label='Magnitude', color='purple', linewidth=2)
        self.axes[3].set_ylabel("Total Acceleration (g)")
        self.axes[3].set_xlabel("Sample")
        self.axes[3].grid(True, alpha=0.3)
        self.axes[3].legend(loc='upper right')
        self.axes[3].set_xlim(0, PLOT_WINDOW)
        self.axes[3].set_ylim(0, 3)
        
        # Pre-create marker artists for each axis and marker type
        self._create_marker_pools()
        
        # Status text
        self.status_text = self.fig.text(0.02, 0.95, "Status: Waiting for data...", fontsize=12, 
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue"))
        
        # Detection status text (larger and more prominent)
        self.detection_text = self.fig.text(0.5, 0.02, "", fontsize=16, ha='center',
                                          bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="black"))
        
        # Add Reset Button
        button_ax = self.fig.add_axes([0.82, 0.94, 0.15, 0.05])  # [left, bottom, width, height] - made slightly larger
        self.reset_button = Button(button_ax, 'RESET\nSEQUENCE', color='lightcoral', hovercolor='red')
        self.reset_button.on_clicked(self.on_reset_button_click)
    
    def _create_marker_pools(self):
        """Simplified marker creation for better performance"""
        # We'll create markers dynamically instead of pre-creating pools
        self.marker_artists = []
    
    def connect_serial(self):
        """Connect to serial port"""
        try:
            print(f"Connecting to {SERIAL_PORT}...")
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"✅ Connected to {SERIAL_PORT}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def parse_serial_data(self, line):
        """Parse incoming serial data line"""
        try:
            # Handle the new 9-value format: acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
            parts = line.strip().split(',')
            if len(parts) == 9:
                acc_x = float(parts[0])
                acc_y = float(parts[1])
                acc_z = float(parts[2])
                gyro_x = float(parts[3])
                gyro_y = float(parts[4])
                gyro_z = float(parts[5])
                mag_x = float(parts[6])
                mag_y = float(parts[7])
                mag_z = float(parts[8])
                return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
            # Fallback to 3-value format for compatibility
            elif len(parts) == 3:
                acc_x = float(parts[0])
                acc_y = float(parts[1])
                acc_z = float(parts[2])
                return acc_x, acc_y, acc_z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        except ValueError:
            pass
        return None
    
    def add_data_point(self, acc_x, acc_y, acc_z, gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, mag_x=0.0, mag_y=0.0, mag_z=0.0):
        """Add new data point to buffer"""
        self.data_buffer['acc_x'].append(acc_x)
        self.data_buffer['acc_y'].append(acc_y)
        self.data_buffer['acc_z'].append(acc_z)
        self.data_buffer['gyro_x'].append(gyro_x)
        self.data_buffer['gyro_y'].append(gyro_y)
        self.data_buffer['gyro_z'].append(gyro_z)
        self.data_buffer['mag_x'].append(mag_x)
        self.data_buffer['mag_y'].append(mag_y)
        self.data_buffer['mag_z'].append(mag_z)
        self.data_buffer['timestamp'].append(time.time())
        self.sample_count += 1
        
        # Perform takeoff detection
        self.detect_takeoff_realtime()
    
    def detect_takeoff_realtime(self):
        """Real-time takeoff detection with ENHANCED 9D IMU validation"""
        if len(self.data_buffer['acc_x']) < RIPPLE_WINDOW:
            return
        
        # Convert current buffer to arrays for analysis
        acc_x = np.array(list(self.data_buffer['acc_x']))
        acc_y = np.array(list(self.data_buffer['acc_y']))
        acc_z = np.array(list(self.data_buffer['acc_z']))
        gyro_x = np.array(list(self.data_buffer['gyro_x']))
        gyro_y = np.array(list(self.data_buffer['gyro_y']))
        gyro_z = np.array(list(self.data_buffer['gyro_z']))
        mag_x = np.array(list(self.data_buffer['mag_x']))
        mag_y = np.array(list(self.data_buffer['mag_y']))
        mag_z = np.array(list(self.data_buffer['mag_z']))
        
        # Calculate magnitude and variance
        magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        recent_mag = magnitude[-10:]
        recent_var_z = np.var(acc_z[-RIPPLE_WINDOW:])
        recent_z_abs = np.abs(acc_z[-10:])
        recent_x_abs = np.abs(acc_x[-10:])
        recent_y_abs = np.abs(acc_y[-10:])
        recent_z_raw = acc_z[-10:]  # Keep raw Z values to check positive/negative
        
        # === ENHANCED 9D IMU CALCULATIONS ===
        # Get recent data for analysis (last N samples)
        recent_z_raw = acc_z[-RIPPLE_WINDOW:]
        recent_z_abs = [abs(z) for z in recent_z_raw]
        recent_var_z = np.var(recent_z_raw) if len(recent_z_raw) >= RIPPLE_WINDOW else 0
        
        # Calculate gyroscope magnitude (angular velocity magnitude)
        recent_gyro_x = gyro_x[-RIPPLE_WINDOW:]
        recent_gyro_y = gyro_y[-RIPPLE_WINDOW:]
        recent_gyro_z = gyro_z[-RIPPLE_WINDOW:]
        gyro_magnitude_recent = np.sqrt(recent_gyro_x**2 + recent_gyro_y**2 + recent_gyro_z**2)
        mean_gyro_magnitude = np.mean(gyro_magnitude_recent) if len(gyro_magnitude_recent) > 0 else 0
        current_gyro_magnitude = np.sqrt(gyro_x[-1]**2 + gyro_y[-1]**2 + gyro_z[-1]**2)
        
        # Calculate magnetometer stability (standard deviation of each axis)
        recent_mag_x = mag_x[-RIPPLE_WINDOW:]
        recent_mag_y = mag_y[-RIPPLE_WINDOW:]
        recent_mag_z_mag = mag_z[-RIPPLE_WINDOW:]  # renamed to avoid confusion with acc_z
        mag_x_std = np.std(recent_mag_x) if len(recent_mag_x) > 0 else 0
        mag_y_std = np.std(recent_mag_y) if len(recent_mag_y) > 0 else 0
        mag_z_std = np.std(recent_mag_z_mag) if len(recent_mag_z_mag) > 0 else 0
        max_mag_std = max(mag_x_std, mag_y_std, mag_z_std)
        
        # Print current sensor readings and calculated values every 10 samples for clarity
        if self.sample_count % 10 == 0:
            current_acc_x, current_acc_y, current_acc_z = acc_x[-1], acc_y[-1], acc_z[-1]
            current_gyro_x, current_gyro_y, current_gyro_z = gyro_x[-1], gyro_y[-1], gyro_z[-1] 
            current_mag_x, current_mag_y, current_mag_z = mag_x[-1], mag_y[-1], mag_z[-1]
            z_mean = np.mean(recent_z_raw) if len(recent_z_raw) > 0 else 0
            z_abs_mean = np.mean(recent_z_abs) if len(recent_z_abs) > 0 else 0
            
            print(f"📊 ACC: X={current_acc_x:.3f}, Y={current_acc_y:.3f}, Z={current_acc_z:.3f} | "
                  f"GYRO: X={current_gyro_x:.3f}, Y={current_gyro_y:.3f}, Z={current_gyro_z:.3f} | "
                  f"MAG: X={current_mag_x:.3f}, Y={current_mag_y:.3f}, Z={current_mag_z:.3f}")
            print(f"🔍 Z_var={recent_var_z:.6f}, Z_mean={z_mean:.3f}, |Z|_mean={z_abs_mean:.3f}, Step={self.sequence_step}")
            print(f"🎯 GYRO_mag={current_gyro_magnitude:.3f}, MAG_std={max_mag_std:.3f}")
            
            # Show detection method being used
            if USE_INDIVIDUAL_AXIS:
                print(f"🔧 Using INDIVIDUAL AXIS method - ACC_thr={INDIVIDUAL_ACC_THRESHOLD}, GYRO_thr={INDIVIDUAL_GYRO_THRESHOLD}")
            else:
                print(f"🔧 Using VARIANCE method - RIPPLE_thr={RIPPLE_THRESHOLD:.6f}")
        
        current_sample = self.sample_count
        
        # === DETECTION METHOD SELECTION ===
        if USE_INDIVIDUAL_AXIS:
            # INDIVIDUAL AXIS APPROACH
            acc_movement = (abs(acc_x[-1]) > INDIVIDUAL_ACC_THRESHOLD or 
                           abs(acc_y[-1]) > INDIVIDUAL_ACC_THRESHOLD or 
                           abs(acc_z[-1]) > INDIVIDUAL_ACC_THRESHOLD)
            motor_running = acc_movement  # For individual axis, movement = motors
            z_takeoff_detected = acc_z[-1] > Z_HIGH_THRESHOLD
            z_stabilized = abs(acc_z[-1]) < Z_STABLE_MAX
            
        else:
            # VARIANCE APPROACH (Original)
            accel_variance = np.var(list(self.data_buffer['acc_x'])[-10:]) + np.var(list(self.data_buffer['acc_y'])[-10:]) + np.var(list(self.data_buffer['acc_z'])[-10:])
            acc_movement = accel_variance > 0.001
            motor_running = recent_var_z > RIPPLE_THRESHOLD
            z_takeoff_detected = np.mean(recent_z_raw) > Z_HIGH_THRESHOLD
            z_stabilized = np.mean(recent_z_abs) < Z_STABLE_MAX
        
        # ENHANCED 9D SEQUENCE VALIDATION
        if self.sequence_step == 1:
            # Step 1: Wait for ANY movement + ENHANCED 9D validation
            # Enhanced conditions: Gyroscope stationary + Magnetometer stable
            gyro_stationary = mean_gyro_magnitude < GYRO_STATIONARY_THRESHOLD
            mag_stable = max_mag_std < MAG_STABILITY_THRESHOLD
            
            # More realistic threshold - allow for normal sensor noise
            if acc_movement and gyro_stationary and mag_stable:
                # Movement detected with stable rotation and magnetic field - advance to step 2
                self.sequence_step = 2
                self.step_start_sample = current_sample
                self.transition_idx = len(acc_x) - 1
                self.takeoff_state = 'ripples'
                
                if USE_INDIVIDUAL_AXIS:
                    print(f"✓ Step 2: Enhanced movement detected (individual axis method, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD})")
                else:
                    accel_variance = np.var(list(self.data_buffer['acc_x'])[-10:]) + np.var(list(self.data_buffer['acc_y'])[-10:]) + np.var(list(self.data_buffer['acc_z'])[-10:])
                    print(f"✓ Step 2: Enhanced movement detected (acc_var={accel_variance:.6f}, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD})")
                
                self.add_detection_marker('motor_start', len(acc_x) - 1)
            elif acc_movement and not gyro_stationary:
                print(f"🚫 Step 1: Manual handling detected (gyro_mag={mean_gyro_magnitude:.3f} > {GYRO_STATIONARY_THRESHOLD}) - ignoring movement")
            elif acc_movement and not mag_stable:
                print(f"🚫 Step 1: Magnetic interference detected (mag_std={max_mag_std:.3f} > {MAG_STABILITY_THRESHOLD}) - ignoring movement")
        
        elif self.sequence_step == 2:
            # Step 2: Stay in ripples and wait for takeoff + ENHANCED 9D validation (RELAXED)
            # Relaxed thresholds for motor vibration tolerance
            gyro_acceptable = mean_gyro_magnitude < (GYRO_STATIONARY_THRESHOLD * 1.5)  # 0.15
            mag_acceptable = max_mag_std < (MAG_STABILITY_THRESHOLD * 1.5)  # 1.5
            
            if motor_running and gyro_acceptable and mag_acceptable:
                # Check if Z-axis goes POSITIVE (upward) for takeoff
                if z_takeoff_detected:
                    self.sequence_step = 3
                    self.step_start_sample = current_sample
                    self.z_high_detected = True
                    self.takeoff_idx = len(acc_x) - 1
                    self.takeoff_state = 'takeoff'
                    
                    if USE_INDIVIDUAL_AXIS:
                        print(f"✓ Step 3: Enhanced takeoff detected (ACC_Z={acc_z[-1]:.3f}>{Z_HIGH_THRESHOLD}, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD*1.5:.3f}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD*1.5:.3f})")
                    else:
                        z_mean = np.mean(recent_z_raw)
                        print(f"✓ Step 3: Enhanced takeoff detected (Z={z_mean:.3f}>{Z_HIGH_THRESHOLD}, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD*1.5:.3f}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD*1.5:.3f})")
                    
                    self.add_detection_marker('takeoff', len(acc_x) - 1)
                # Still in ripples - this is GOOD, stay here until takeoff
            elif motor_running and not gyro_acceptable:
                print(f"🚫 Step 2: High rotation detected (gyro_mag={mean_gyro_magnitude:.3f} > {GYRO_STATIONARY_THRESHOLD*1.5:.3f}) - possible manual handling")
                # Don't reset immediately, allow some motor-induced rotation
            elif motor_running and not mag_acceptable:
                print(f"🚫 Step 2: Magnetic instability detected (mag_std={max_mag_std:.3f} > {MAG_STABILITY_THRESHOLD*1.5:.3f}) - possible interference")
                # Don't reset immediately, allow some motor-induced magnetic variation
            else:
                # Lost ripples/movement - but be more tolerant, don't reset immediately
                samples_in_step = self.sample_count - self.step_start_sample
                if samples_in_step > 100:  # Only reset after 100 samples of no movement
                    if USE_INDIVIDUAL_AXIS:
                        print(f"⚠️  Reset: Lost movement in step 2 (no significant axis values, samples={samples_in_step})")
                    else:
                        print(f"⚠️  Reset: Lost ripples in step 2 (variance={recent_var_z:.6f} < {RIPPLE_THRESHOLD}, samples={samples_in_step})")
                    self.reset_detection("Lost movement/ripples in step 2 for extended period")
                # Otherwise, just wait for movement/ripples to return
        
        elif self.sequence_step == 3:
            # Step 3: Z-axis must go HIGH then return to RIPPLES + ENHANCED 9D validation (MODERATE)
            # Moderate thresholds - allow takeoff-induced movement but detect manual handling
            gyro_moderate = mean_gyro_magnitude < (GYRO_STATIONARY_THRESHOLD * 2.0)  # 0.2
            mag_moderate = max_mag_std < (MAG_STABILITY_THRESHOLD * 2.0)  # 2.0
            
            if self.z_high_detected:
                # Z was positive, now check if it goes NEGATIVE and returns to ripples/movement
                if USE_INDIVIDUAL_AXIS:
                    z_returned = acc_z[-1] < -Z_RETURN_THRESHOLD
                    motors_still_running = acc_movement  # Individual axis movement
                else:
                    z_mean = np.mean(recent_z_raw)
                    z_returned = z_mean < -Z_RETURN_THRESHOLD
                    motors_still_running = recent_var_z > RIPPLE_THRESHOLD
                
                if z_returned and motors_still_running and gyro_moderate and mag_moderate:
                    # Z returned to negative with movement/ripples - advance to step 4
                    self.sequence_step = 4
                    self.step_start_sample = current_sample  # Reference only
                    self.stable_count = 0  # Reset stability counter
                    
                    if USE_INDIVIDUAL_AXIS:
                        print(f"✓ Step 4: Enhanced stabilizing (ACC_Z={acc_z[-1]:.3f}<-{Z_RETURN_THRESHOLD}, movement=True, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD*2.0:.3f}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD*2.0:.3f})")
                    else:
                        z_mean = np.mean(recent_z_raw)
                        print(f"✓ Step 4: Enhanced stabilizing (Z={z_mean:.3f}<-{Z_RETURN_THRESHOLD}, var={recent_var_z:.6f}>{RIPPLE_THRESHOLD}, gyro_mag={mean_gyro_magnitude:.3f}<{GYRO_STATIONARY_THRESHOLD*2.0:.3f}, mag_std={max_mag_std:.3f}<{MAG_STABILITY_THRESHOLD*2.0:.3f})")
                
                elif not motors_still_running:
                    # Lost movement/ripples completely - INVALID SEQUENCE
                    if USE_INDIVIDUAL_AXIS:
                        print(f"⚠️  Reset: Lost movement in step 3 (no significant axis values)")
                    else:
                        z_mean = np.mean(recent_z_raw)
                        print(f"⚠️  Reset: Lost ripples in step 3 (variance={recent_var_z:.6f} < {RIPPLE_THRESHOLD}, Z={z_mean:.3f})")
                    self.reset_detection("Lost movement/ripples in step 3")
                elif not gyro_moderate:
                    # Excessive rotation detected - likely manual handling
                    print(f"⚠️  Reset: Excessive rotation in step 3 (gyro_mag={mean_gyro_magnitude:.3f} > {GYRO_STATIONARY_THRESHOLD*2.0:.3f}) - manual handling detected")
                    self.reset_detection("Excessive rotation during takeoff - manual handling")
                elif not mag_moderate:
                    # Excessive magnetic disturbance - likely manual movement
                    print(f"⚠️  Reset: Magnetic disturbance in step 3 (mag_std={max_mag_std:.3f} > {MAG_STABILITY_THRESHOLD*2.0:.3f}) - external interference")
                    self.reset_detection("Magnetic disturbance during takeoff")
                # Still high or transitioning - CAN WAIT INDEFINITELY if sensors are stable
            else:
                # Should not reach here, but safety check
                self.reset_detection("Step 3 logic error")
        
        elif self.sequence_step == 4:
            # Step 4: Must be STABLE in ripples + ENHANCED 9D validation (STRICT)
            # Strict thresholds - confirm stable hover with minimal rotation and magnetic stability
            gyro_stable = mean_gyro_magnitude < GYRO_STATIONARY_THRESHOLD  # 0.1 (back to strict)
            mag_stable = max_mag_std < MAG_STABILITY_THRESHOLD  # 1.0 (back to strict)
            
            if USE_INDIVIDUAL_AXIS:
                movement_present = acc_movement
                z_stable = z_stabilized  # abs(acc_z[-1]) < Z_STABLE_MAX
            else:
                movement_present = recent_var_z > RIPPLE_THRESHOLD
                z_stable = np.mean(recent_z_abs) < Z_STABLE_MAX
            
            if movement_present and z_stable and gyro_stable and mag_stable:
                # Count consecutive stable samples with all sensors confirming stability
                self.stable_count += 1
                
                # Check if we've had enough consecutive stable samples (minimum confirmation)
                if self.stable_count >= self.min_stable_samples:
                    # SUCCESSFUL TAKEOFF SEQUENCE COMPLETED!
                    if self.takeoff_state != 'stabilized':  # Only validate once
                        self.takeoff_state = 'stabilized'
                        self.stabilization_idx = len(acc_x) - 1
                        
                        if USE_INDIVIDUAL_AXIS:
                            print(f"🎉 ENHANCED TAKEOFF SEQUENCE COMPLETED! (stable_count={self.stable_count}, ACC_Z={abs(acc_z[-1]):.3f}<{Z_STABLE_MAX}, gyro_mag={mean_gyro_magnitude:.3f}, mag_std={max_mag_std:.3f})")
                        else:
                            z_abs_mean = np.mean(recent_z_abs)
                            print(f"🎉 ENHANCED TAKEOFF SEQUENCE COMPLETED! (stable_count={self.stable_count}, var={recent_var_z:.6f}, Z_abs={z_abs_mean:.3f}, gyro_mag={mean_gyro_magnitude:.3f}, mag_std={max_mag_std:.3f})")
                        
                        self.add_detection_marker('stabilization', len(acc_x) - 1)
                        self.sequence_step = 5  # Mark as completed to stop further processing
            elif not z_stable:
                # Big Z movement again - reset stability counter but don't fail yet
                self.stable_count = 0
                if USE_INDIVIDUAL_AXIS:
                    print(f"⚠️  Stability reset: Z movement detected (|ACC_Z|={abs(acc_z[-1]):.3f} > {Z_STABLE_MAX})")
                else:
                    z_abs_mean = np.mean(recent_z_abs)
                    print(f"⚠️  Stability reset: Z movement detected (|Z|={z_abs_mean:.3f} > {Z_STABLE_MAX})")
            elif not movement_present:
                # Lost movement/ripples in stabilization - INVALID
                if USE_INDIVIDUAL_AXIS:
                    print(f"⚠️  Reset: Lost movement in step 4 (no significant axis values)")
                else:
                    print(f"⚠️  Reset: Lost ripples in step 4 (variance={recent_var_z:.6f} < {RIPPLE_THRESHOLD})")
                self.reset_detection("Lost movement/ripples during stabilization")
            elif not gyro_stable:
                # Excessive rotation during stabilization - likely manual handling
                print(f"⚠️  Reset: Rotation detected during stabilization (gyro_mag={mean_gyro_magnitude:.3f} > {GYRO_STATIONARY_THRESHOLD}) - manual handling")
                self.reset_detection("Rotation detected during stabilization - manual handling")
            elif not mag_stable:
                # Magnetic instability during stabilization
                print(f"⚠️  Reset: Magnetic instability during stabilization (mag_std={max_mag_std:.3f} > {MAG_STABILITY_THRESHOLD}) - interference")
                self.reset_detection("Magnetic instability during stabilization")
        
        elif self.sequence_step == 5:
            # Sequence completed - no further processing needed
            pass
    
    def reset_detection(self, reason):
        """Reset detection to start over from step 1"""
        # Don't print here - diagnostic prints already show the reason with values
        
        self.sequence_step = 1
        self.takeoff_state = 'waiting'
        self.transition_idx = -1
        self.takeoff_idx = -1
        self.stabilization_idx = -1
        self.z_high_detected = False
        self.step_start_sample = self.sample_count
        self.stable_count = 0  # Reset stability counter
        self.reset_count += 1
        
        # Clear detection markers
        self.detection_markers = {
            'motor_start': [],
            'takeoff': [],
            'stabilization': []
        }
    
    def add_detection_marker(self, event_type, sample_idx):
        """Add a detection marker for visualization - simplified"""
        # Store the marker info relative to current buffer position
        buffer_position = len(self.data_buffer['acc_x']) - 1
        self.detection_markers[event_type].append({
            'sample_idx': sample_idx,
            'buffer_pos': buffer_position,
            'timestamp': time.time()
        })
        
        # Keep only recent markers (last 5 of each type for performance)
        if len(self.detection_markers[event_type]) > 5:
            self.detection_markers[event_type].pop(0)
    
    def on_reset_button_click(self, event):
        """Callback for the reset button"""
        self.reset_detection("User requested reset")
    
    def read_serial_data(self):
        """Thread function to read serial data"""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8')
                    data = self.parse_serial_data(line)
                    if data:
                        self.add_data_point(*data)
                    # Remove parsing error message to keep console clean
                else:
                    time.sleep(0.01)  # Small delay when no data available
            except serial.SerialException as e:
                print(f"Serial connection error: {e}")
                break  # Break only on serial connection errors
            except Exception as e:
                print(f"Serial data processing error: {e}")
                # Don't break - continue reading even if there's a processing error
                time.sleep(0.1)
        
        print("Serial reading thread stopped")
    
    def console_input_loop(self):
        """Loop to accept console input for manual data entry"""
        print("💡 Console input ready")
        while self.running:
            try:
                line = input()
                if line.strip():
                    data = self.parse_serial_data(line)
                    if data:
                        self.add_data_point(*data)
                    # Remove parsing error message to keep console clean
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"Input error: {e}")
    
    def simulate_data_from_csv(self):
        """Simulate live data from CSV file for testing"""
        data = pd.read_csv(CSV_FILE)
        for i, row in data.iterrows():
            if not self.running:
                break
            # Assuming CSV has all 9D data now
            self.add_data_point(row['acc_x'], row['acc_y'], row['acc_z'],
                                 row['gyro_x'], row['gyro_y'], row['gyro_z'],
                                 row['mag_x'], row['mag_y'], row['mag_z'])
            time.sleep(0.05)  # Simulate 20Hz data rate
    
    def update_plot(self, frame):
        """Update plot with latest data - OPTIMIZED for speed"""
        try:
            if len(self.data_buffer['acc_x']) == 0:
                return list(self.lines.values())
            
            # Check if axes are still valid
            if not all(ax is not None for ax in self.axes):
                return list(self.lines.values())
            
            # Get recent data for plotting
            plot_size = min(PLOT_WINDOW, len(self.data_buffer['acc_x']))
            x_data = list(range(plot_size))
            
            # Update accelerometer plot (Plot 1)
            if plot_size > 0:
                acc_x_data = list(self.data_buffer['acc_x'])[-plot_size:]
                acc_y_data = list(self.data_buffer['acc_y'])[-plot_size:]
                acc_z_data = list(self.data_buffer['acc_z'])[-plot_size:]
                
                self.lines['acc_x'].set_data(x_data, acc_x_data)
                self.lines['acc_y'].set_data(x_data, acc_y_data)
                self.lines['acc_z'].set_data(x_data, acc_z_data)
                
                # Auto-scale accelerometer Y-axis
                all_acc_data = acc_x_data + acc_y_data + acc_z_data
                if all_acc_data:
                    margin = 0.1
                    y_min, y_max = min(all_acc_data), max(all_acc_data)
                    y_range = max(0.1, y_max - y_min)
                    self.axes[0].set_ylim(y_min - margin * y_range, y_max + margin * y_range)
            
            # Update gyroscope plot (Plot 2)
            if plot_size > 0:
                gyro_x_data = list(self.data_buffer['gyro_x'])[-plot_size:]
                gyro_y_data = list(self.data_buffer['gyro_y'])[-plot_size:]
                gyro_z_data = list(self.data_buffer['gyro_z'])[-plot_size:]
                
                self.lines['gyro_x'].set_data(x_data, gyro_x_data)
                self.lines['gyro_y'].set_data(x_data, gyro_y_data)
                self.lines['gyro_z'].set_data(x_data, gyro_z_data)
                
                # Auto-scale gyroscope Y-axis
                all_gyro_data = gyro_x_data + gyro_y_data + gyro_z_data
                if all_gyro_data:
                    margin = 0.1
                    y_min, y_max = min(all_gyro_data), max(all_gyro_data)
                    y_range = max(0.1, y_max - y_min)
                    self.axes[1].set_ylim(y_min - margin * y_range, y_max + margin * y_range)
            
            # Update magnetometer plot (Plot 3)
            if plot_size > 0:
                mag_x_data = list(self.data_buffer['mag_x'])[-plot_size:]
                mag_y_data = list(self.data_buffer['mag_y'])[-plot_size:]
                mag_z_data = list(self.data_buffer['mag_z'])[-plot_size:]
                
                self.lines['mag_x'].set_data(x_data, mag_x_data)
                self.lines['mag_y'].set_data(x_data, mag_y_data)
                self.lines['mag_z'].set_data(x_data, mag_z_data)
            
            # Update magnitude plot (Plot 4)
            if plot_size > 0:
                acc_x = np.array(acc_x_data)
                acc_y = np.array(acc_y_data)
                acc_z = np.array(acc_z_data)
                magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
                
                self.lines['magnitude'].set_data(x_data, magnitude)
                if len(magnitude) > 0:
                    self.axes[3].set_ylim(0, max(1.0, np.max(magnitude) * 1.1))
            
            # Simplified status update (no complex markers for better performance)
            status_msg = f"Status: {self.takeoff_state.capitalize()} | Step: {self.sequence_step}/4 | Samples: {self.sample_count}"
            if self.reset_count > 0:
                status_msg += f" | Resets: {self.reset_count}"
            
            self.status_text.set_text(status_msg)
            
            # Update detection status
            if self.sequence_step == 1:
                reset_info = f" (Resets: {self.reset_count})" if self.reset_count > 0 else ""
                self.detection_text.set_text(f"Step 1/4: Waiting for movement{reset_info}")
                self.detection_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor="lightgray", edgecolor="gray"))
            elif self.sequence_step == 2:
                elapsed = self.sample_count - self.step_start_sample
                self.detection_text.set_text(f"Step 2/4: In Ripples - Waiting for Takeoff ({elapsed} samples)")
                self.detection_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor="orange", edgecolor="darkorange"))
            elif self.sequence_step == 3:
                elapsed = self.sample_count - self.step_start_sample
                if self.z_high_detected:
                    self.detection_text.set_text(f"Step 3/4: Z-High - Waiting for Return to Ripples ({elapsed} samples)")
                else:
                    self.detection_text.set_text(f"Step 3/4: Waiting for Z-Axis Takeoff ({elapsed} samples)")
                self.detection_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor="yellow", edgecolor="orange"))
            elif self.sequence_step == 4:
                self.detection_text.set_text(f"Step 4/4: Stable Count: {self.stable_count}/{self.min_stable_samples}")
                self.detection_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor="lightgreen", edgecolor="green"))
            
            # Override with final states
            if self.takeoff_state == 'stabilized':
                self.detection_text.set_text("*** TAKEOFF SEQUENCE VALIDATED! *** (Click RESET to start new detection)")
                self.detection_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor="green", edgecolor="darkgreen"))
            
            return list(self.lines.values()) + [self.status_text, self.detection_text]
        
        except Exception as e:
            print(f"Error in update_plot: {e}")
            return list(self.lines.values())
    
    def start(self):
        """Start real-time detection"""
        self.running = True
        
        if USE_LIVE_DATA:
            if self.connect_serial():
                print("✅ Live data mode")
                # Show detection thresholds for tuning
                if USE_INDIVIDUAL_AXIS:
                    print(f"📊 INDIVIDUAL AXIS METHOD - ACC_thr={INDIVIDUAL_ACC_THRESHOLD}g, GYRO_thr={INDIVIDUAL_GYRO_THRESHOLD}rad/s, MAG_thr={INDIVIDUAL_MAG_THRESHOLD}")
                    print(f"📊 Z_takeoff={Z_HIGH_THRESHOLD}g, Z_return={Z_RETURN_THRESHOLD}g, Z_stable={Z_STABLE_MAX}g")
                else:
                    print(f"📊 VARIANCE METHOD - RIPPLE={RIPPLE_THRESHOLD:.6f}, Z_HIGH={Z_HIGH_THRESHOLD}, Z_RETURN={Z_RETURN_THRESHOLD}, Z_STABLE={Z_STABLE_MAX}")
                # Start serial reading thread
                serial_thread = threading.Thread(target=self.read_serial_data)
                serial_thread.daemon = True
                serial_thread.start()
            else:
                print("❌ Using manual input mode")
            
            # Always start console input thread for manual data entry
            console_thread = threading.Thread(target=self.console_input_loop)
            console_thread.daemon = True  
            console_thread.start()
            
        else:
            # Use CSV simulation
            csv_thread = threading.Thread(target=self.simulate_data_from_csv)
            csv_thread.daemon = True
            csv_thread.start()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, blit=False, 
                                    cache_frame_data=False, save_count=100)
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        """Stop real-time detection"""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

def main():
    detector = RealTimeDroneDetector()
    try:
        detector.start()
    except KeyboardInterrupt:
        print("\nStopping real-time detection...")
        detector.stop()

if __name__ == "__main__":
    main()

