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
SERIAL_PORT = 'COM5'                 # Serial port for live data
BAUD_RATE = 115200                   # Baud rate for serial communication
#CSV_FILE = 'data_with_ripples.csv'  # Backup data file for testing
CSV_FILE = 'imu_data_20250718_100137.csv'  # Backup data file for testing
ACC_COLS = ['acc_x', 'acc_y', 'acc_z']  # Names of accelerometer columns
RIPPLE_WINDOW = 20                   # Window size for ripple detection
Z_WINDOW = 50                        # Window size for z-change detection
RIPPLE_THRESHOLD = 0.0005             # Lowered variance threshold for ripples
Z_CHANGE_THRESHOLD = 0.5         # Minimum z-change for takeoff detection
BUFFER_SIZE = 500                    # Number of samples to keep in buffer
PLOT_WINDOW = 200                    # Number of samples to show in plot
USE_LIVE_DATA = True                 # Set to False to use CSV file instead

# Step 3 Detection Thresholds
Z_HIGH_THRESHOLD = 0.1                # Threshold for detecting Z-axis going high (Step 2->3)
Z_RETURN_THRESHOLD = 0.1             # Threshold for detecting Z-axis returning from high (Step 3->4)
XY_RIPPLE_MAX = 0.4                   # Maximum allowed X/Y values during takeoff (should stay in ripple range)
Z_STABLE_MAX = 0.2                    # Maximum Z-axis absolute value allowed in Step 4 for stability

class RealTimeDroneDetector:
    def __init__(self):
        self.data_buffer = {
            'acc_x': deque(maxlen=BUFFER_SIZE),
            'acc_y': deque(maxlen=BUFFER_SIZE),
            'acc_z': deque(maxlen=BUFFER_SIZE),
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
        
        # Setup plot
        self.fig, self.axes = plt.subplots(4, 1, figsize=(8, 6))
        self.fig.suptitle("Real-Time Drone Takeoff Detection", fontsize=16)
        
        # Initialize plot lines
        self.lines = {}
        self.time_data = deque(maxlen=PLOT_WINDOW)
        
        # Detection markers for visualization
        self.detection_markers = {
            'motor_start': [],
            'takeoff': [],
            'stabilization': []
        }
        
        for i, col in enumerate(ACC_COLS):
            self.lines[col], = self.axes[i].plot([], [], label=col.upper(), linewidth=1)
            self.axes[i].set_ylabel(f"{col.upper()} (m/s²)")
            self.axes[i].grid(True)
            self.axes[i].legend()
            self.axes[i].set_xlim(0, PLOT_WINDOW)
            self.axes[i].set_ylim(-3, 3)
        
        # Magnitude plot
        self.lines['magnitude'], = self.axes[3].plot([], [], label='Magnitude', color='purple', linewidth=1)
        self.axes[3].set_ylabel("Magnitude (m/s²)")
        self.axes[3].set_xlabel("Sample")
        self.axes[3].grid(True)
        self.axes[3].legend()
        self.axes[3].set_xlim(0, PLOT_WINDOW)
        self.axes[3].set_ylim(0, 3)
        
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
    
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
            return True
        except Exception as e:
            print(f"Failed to connect to {SERIAL_PORT}: {e}")
            return False
    
    def parse_serial_data(self, line):
        """Parse incoming serial data line"""
        try:
            # Assuming data format: "acc_x,acc_y,acc_z\n"
            parts = line.strip().split(',')
            if len(parts) == 3:
                acc_x = float(parts[0])
                acc_y = float(parts[1])
                acc_z = float(parts[2])
                return acc_x, acc_y, acc_z
        except ValueError:
            pass
        return None
    
    def add_data_point(self, acc_x, acc_y, acc_z):
        """Add new data point to buffer"""
        self.data_buffer['acc_x'].append(acc_x)
        self.data_buffer['acc_y'].append(acc_y)
        self.data_buffer['acc_z'].append(acc_z)
        self.data_buffer['timestamp'].append(time.time())
        self.sample_count += 1
        
        # Perform takeoff detection
        self.detect_takeoff_realtime()
    
    def detect_takeoff_realtime(self):
        """Real-time takeoff detection with strict sequence validation"""
        if len(self.data_buffer['acc_x']) < RIPPLE_WINDOW:
            return
        
        # Convert current buffer to arrays for analysis
        acc_x = np.array(list(self.data_buffer['acc_x']))
        acc_y = np.array(list(self.data_buffer['acc_y']))
        acc_z = np.array(list(self.data_buffer['acc_z']))
        
        # Calculate magnitude and variance
        magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        recent_mag = magnitude[-10:]
        recent_var_z = np.var(acc_z[-RIPPLE_WINDOW:])
        recent_z_abs = np.abs(acc_z[-10:])
        recent_x_abs = np.abs(acc_x[-10:])
        recent_y_abs = np.abs(acc_y[-10:])
        recent_z_raw = acc_z[-10:]  # Keep raw Z values to check positive/negative
        
        # Get current x, y, z values for printing
        current_x = acc_x[-1]
        current_y = acc_y[-1]
        current_z = acc_z[-1]
        
        current_sample = self.sample_count
        
        # Debug: Print current step every 100 samples
        if self.sample_count % 100 == 0:
            print(f"🔍 DEBUG: Currently in Step {self.sequence_step}/4 at sample {current_sample}")
        
        # STRICT SEQUENCE VALIDATION
        if self.sequence_step == 1:
            # Step 1: Must be in ZEROS state
            if np.mean(recent_mag) < 0.01 and recent_var_z < RIPPLE_THRESHOLD:
                # Still in zeros - good
                pass
            elif np.mean(recent_mag) > 0.01 and recent_var_z > RIPPLE_THRESHOLD:
                # Transition to ripples - advance to step 2 (NO TIME CONSTRAINT)
                self.sequence_step = 2
                self.step_start_sample = current_sample  # Reference only
                self.transition_idx = len(acc_x) - 1
                self.takeoff_state = 'ripples'
                print(f"📍 ENTERED STEP 1")
                print(f"✓ Step 1->2: Zero to Ripples transition at sample {current_sample} (IMMEDIATE)")
                print(f"   XYZ Values: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
                self.add_detection_marker('motor_start', len(acc_x) - 1)
            else:
                # Invalid state - reset
                self.reset_detection("Invalid transition from zeros")
        
        elif self.sequence_step == 2:
            # Step 2: Must stay in RIPPLES - can wait FOREVER for Z takeoff
            if recent_var_z > RIPPLE_THRESHOLD:
                # Check if Z-axis goes POSITIVE (upward) AND X,Y stay in ripple range (start of step 3)
                if (np.mean(recent_z_raw) > Z_HIGH_THRESHOLD and 
                    np.mean(recent_x_abs) < XY_RIPPLE_MAX and 
                    np.mean(recent_y_abs) < XY_RIPPLE_MAX):
                    self.sequence_step = 3
                    self.step_start_sample = current_sample  # Reference only
                    self.z_high_detected = True
                    self.takeoff_idx = len(acc_x) - 1
                    self.takeoff_state = 'takeoff'
                    elapsed_time = current_sample - self.step_start_sample
                    print(f"📍 ENTERED STEP 2")
                    print(f"✓ Step 2->3: Ripples to Z-POSITIVE at sample {current_sample} (after {elapsed_time} samples in ripples)")
                    print(f"   XYZ Values: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
                    print(f"   Z-avg=+{np.mean(recent_z_raw):.4f} (positive upward), X/Y in range: x={np.mean(recent_x_abs):.4f}, y={np.mean(recent_y_abs):.4f}")
                    self.add_detection_marker('takeoff', len(acc_x) - 1)
                # Still in ripples - PERFECTLY FINE, can wait indefinitely
            else:
                # Lost ripples without going to Z-high - INVALID SEQUENCE
                self.reset_detection("Lost ripples in step 2 without Z-axis takeoff")
        
        elif self.sequence_step == 3:
            # Step 3: Z-axis must go HIGH then return to RIPPLES - NO TIME LIMIT!
            if self.z_high_detected:
                # Z was positive, now check if it goes NEGATIVE and returns to ripples
                if np.mean(recent_z_raw) < -Z_RETURN_THRESHOLD and recent_var_z > RIPPLE_THRESHOLD:
                    # Z returned to ripples - advance to step 4
                    self.sequence_step = 4
                    self.step_start_sample = current_sample  # Reference only
                    self.stable_count = 0  # Reset stability counter
                    elapsed_time = current_sample - self.step_start_sample
                    print(f"📍 ENTERED STEP 4")
                    print(f"✓ Step 3->4: Z went NEGATIVE and returned to ripples at sample {current_sample} (Z was positive for {elapsed_time} samples)")
                    print(f"   XYZ Values: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
                    print(f"   Z-avg={np.mean(recent_z_raw):.4f} (negative settling)")
                elif recent_var_z < RIPPLE_THRESHOLD:
                    # Lost ripples completely (no variance) - INVALID SEQUENCE
                    self.reset_detection("Lost ripples in step 3 - variance too low")
                # Still high or transitioning - CAN WAIT INDEFINITELY
            else:
                # Should not reach here, but safety check
                self.reset_detection("Step 3 logic error")
        
        elif self.sequence_step == 4:
            # Step 4: Must be STABLE in ripples (no big changes) - NO TIME LIMIT!
            if recent_var_z > RIPPLE_THRESHOLD and np.mean(recent_z_abs) < Z_STABLE_MAX:
                # Count consecutive stable samples
                self.stable_count += 1
                
                # Check if we've had enough consecutive stable samples (minimum confirmation)
                if self.stable_count >= self.min_stable_samples:
                    # SUCCESSFUL TAKEOFF SEQUENCE COMPLETED!
                    if self.takeoff_state != 'stabilized':  # Only validate once
                        self.takeoff_state = 'stabilized'
                        self.stabilization_idx = len(acc_x) - 1
                        print(f"📍 ENTERED STEP 4 (COMPLETED)")
                        print(f"🎉 TAKEOFF SEQUENCE VALIDATED! Stabilized at sample {current_sample} (after {self.stable_count} stable samples)")
                        print(f"   Final XYZ Values: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
                        self.add_detection_marker('stabilization', len(acc_x) - 1)
                        self.sequence_step = 5  # Mark as completed to stop further processing
            elif np.mean(recent_z_abs) > Z_STABLE_MAX:
                # Big Z movement again - reset stability counter but don't fail yet
                self.stable_count = 0
                print(f"⚠️ Step 4: Z-axis movement detected (|z|={np.mean(recent_z_abs):.4f} > {Z_STABLE_MAX}), resetting stability counter")
                print(f"   XYZ Values: x={current_x:.4f}, y={current_y:.4f}, z={current_z:.4f}")
            elif recent_var_z < RIPPLE_THRESHOLD:
                # Lost ripples in stabilization - INVALID
                self.reset_detection("Lost ripples during stabilization")
        
        elif self.sequence_step == 5:
            # Sequence completed - no further processing needed
            pass
    
    def reset_detection(self, reason):
        """Reset detection to start over from step 1"""
        print(f"📍 BACK TO STEP 1")
        print(f"❌ SEQUENCE RESET: {reason} - Starting over from step 1")
        print(f"🕒 Reset time: {time.strftime('%H:%M:%S')}")
        
        self.sequence_step = 1
        self.takeoff_state = 'waiting'
        self.transition_idx = -1
        self.takeoff_idx = -1
        self.stabilization_idx = -1
        self.z_high_detected = False
        self.step_start_sample = self.sample_count
        self.stable_count = 0  # Reset stability counter
        self.reset_count += 1
        
        # Clear detection markers to show reset
        self.detection_markers = {
            'motor_start': [],
            'takeoff': [],
            'stabilization': []
        }
        
        # Flash the reset button to show action was registered
        if hasattr(self, 'reset_button'):
            original_color = self.reset_button.color
            self.reset_button.color = 'lime'  # Flash green briefly
            self.fig.canvas.draw_idle()
            # Reset color after short delay (handled by button hover system)
            threading.Timer(0.5, lambda: setattr(self.reset_button, 'color', original_color)).start()
    
    def add_detection_marker(self, event_type, sample_idx):
        """Add a detection marker for visualization"""
        # Store the marker info relative to current buffer position
        buffer_position = len(self.data_buffer['acc_x']) - 1
        self.detection_markers[event_type].append({
            'sample_idx': sample_idx,
            'buffer_pos': buffer_position,
            'timestamp': time.time()
        })
        
        # Keep only recent markers (last 10 of each type)
        if len(self.detection_markers[event_type]) > 10:
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
            except Exception as e:
                print(f"Serial read error: {e}")
                break
    
    def simulate_data_from_csv(self):
        """Simulate live data from CSV file for testing"""
        data = pd.read_csv(CSV_FILE)
        for i, row in data.iterrows():
            if not self.running:
                break
            self.add_data_point(row['acc_x'], row['acc_y'], row['acc_z'])
            time.sleep(0.05)  # Simulate 20Hz data rate
    
    def update_plot(self, frame):
        """Update plot with latest data"""
        if len(self.data_buffer['acc_x']) == 0:
            return self.lines.values()
        
        # Get recent data for plotting
        plot_size = min(PLOT_WINDOW, len(self.data_buffer['acc_x']))
        x_data = list(range(plot_size))
        
        # Update acceleration plots
        for i, col in enumerate(ACC_COLS):
            recent_data = list(self.data_buffer[col])[-plot_size:]
            self.lines[col].set_data(x_data, recent_data)
            
            # Auto-scale y-axis
            if recent_data:
                margin = 0.1
                y_min, y_max = min(recent_data), max(recent_data)
                y_range = max(0.1, y_max - y_min)
                self.axes[i].set_ylim(y_min - margin * y_range, y_max + margin * y_range)
        
        # Update magnitude plot
        if plot_size > 0:
            acc_x = np.array(list(self.data_buffer['acc_x'])[-plot_size:])
            acc_y = np.array(list(self.data_buffer['acc_y'])[-plot_size:])
            acc_z = np.array(list(self.data_buffer['acc_z'])[-plot_size:])
            magnitude = np.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
            
            self.lines['magnitude'].set_data(x_data, magnitude)
            if len(magnitude) > 0:
                self.axes[3].set_ylim(0, max(1.0, np.max(magnitude) * 1.1))
        
        # Clear previous markers and add current detection markers
        for ax in self.axes:
            # Clear previous detection lines and annotations
            for line in ax.lines[1:]:  # Keep only the main data line
                if hasattr(line, '_detection_marker'):
                    line.remove()
            # Clear annotations
            for text in ax.texts[:]:
                text.remove()
        
        # Add detection markers to all plots
        current_buffer_size = len(self.data_buffer['acc_x'])
        
        # Motor start markers (orange)
        for marker in self.detection_markers['motor_start']:
            plot_pos = current_buffer_size - (buffer_position := marker['buffer_pos']) + plot_size - 1
            if 0 <= plot_pos < plot_size:
                for ax in self.axes:
                    line = ax.axvline(plot_pos, color='orange', linestyle='--', linewidth=2, alpha=0.7)
                    line._detection_marker = True
                    ax.text(plot_pos, ax.get_ylim()[1] * 0.9, 'MOTOR\nSTART', 
                           ha='center', va='top', fontsize=8, 
                           bbox=dict(boxstyle="round,pad=0.2", facecolor="orange", alpha=0.7))
        
        # Takeoff markers (green)
        for marker in self.detection_markers['takeoff']:
            plot_pos = current_buffer_size - marker['buffer_pos'] + plot_size - 1
            if 0 <= plot_pos < plot_size:
                for ax in self.axes:
                    line = ax.axvline(plot_pos, color='green', linestyle='-', linewidth=3, alpha=0.8)
                    line._detection_marker = True
                    ax.text(plot_pos, ax.get_ylim()[1] * 0.8, 'TAKEOFF\nDETECTED!', 
                           ha='center', va='top', fontsize=10, fontweight='bold',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="green", alpha=0.8, edgecolor="darkgreen"))
        
        # Stabilization markers (blue)
        for marker in self.detection_markers['stabilization']:
            plot_pos = current_buffer_size - marker['buffer_pos'] + plot_size - 1
            if 0 <= plot_pos < plot_size:
                for ax in self.axes:
                    line = ax.axvline(plot_pos, color='blue', linestyle='-.', linewidth=2, alpha=0.7)
                    line._detection_marker = True
                    ax.text(plot_pos, ax.get_ylim()[1] * 0.7, 'STABLE', 
                           ha='center', va='top', fontsize=8,
                           bbox=dict(boxstyle="round,pad=0.2", facecolor="lightblue", alpha=0.7))
        
        # Update status with sequence step
        status_msg = f"Status: {self.takeoff_state.capitalize()} | Step: {self.sequence_step}/4 | Samples: {self.sample_count}"
        if self.transition_idx >= 0:
            status_msg += f" | Motor: {self.transition_idx}"
        if self.takeoff_idx >= 0:
            status_msg += f" | Takeoff: {self.takeoff_idx}"
        if self.stabilization_idx >= 0:
            status_msg += f" | Stable: {self.stabilization_idx}"
        status_msg += f" | Reset Count: {self.reset_count}" # Add reset count to status
        
        self.status_text.set_text(status_msg)
        
        # Update large detection status text with sequence validation
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
    
    def start(self):
        """Start real-time detection"""
        self.running = True
        
        if USE_LIVE_DATA:
            if self.connect_serial():
                # Start serial reading thread
                serial_thread = threading.Thread(target=self.read_serial_data)
                serial_thread.daemon = True
                serial_thread.start()
            else:
                print("Failed to connect to serial port. Using CSV simulation.")
                # Fallback to CSV simulation
                csv_thread = threading.Thread(target=self.simulate_data_from_csv)
                csv_thread.daemon = True
                csv_thread.start()
        else:
            # Use CSV simulation
            csv_thread = threading.Thread(target=self.simulate_data_from_csv)
            csv_thread.daemon = True
            csv_thread.start()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, 
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