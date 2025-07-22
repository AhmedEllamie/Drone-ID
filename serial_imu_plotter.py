import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
from collections import deque
import time
import threading
import queue
import re

class SerialIMUPlotter:
    def __init__(self, window_size=1500):  # Larger buffer for 190 Hz
        self.window_size = window_size
        
        # Data storage
        self.timestamps = deque(maxlen=window_size)
        self.x_data = deque(maxlen=window_size)
        self.y_data = deque(maxlen=window_size)
        self.z_data = deque(maxlen=window_size)
        
        # Statistics
        self.data_count = 0
        self.start_time = None
        self.last_update = None
        self.current_rate = 0
        self.max_rate = 0
        
        # Current values
        self.current_x = 0
        self.current_y = 0 
        self.current_z = 0
        
        # Motion detection
        self.motion_threshold = 0.1
        self.is_motion = False
        
        # Data queue for thread communication
        self.data_queue = queue.Queue()
        self.running = True
        
        # File monitoring
        self.monitor_file = "imu_data.txt"
        self.file_position = 0
        self.file_monitoring = False
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        plt.style.use('dark_background')
        self.fig, (self.ax_main, self.ax_info) = plt.subplots(2, 1, figsize=(14, 10), 
                                                              gridspec_kw={'height_ratios': [4, 1]})
        
        # Main plot setup
        self.ax_main.set_title('🚁 Real-Time IMU Data Stream (190 Hz)', fontsize=16, color='white')
        self.ax_main.set_xlabel('Time (seconds)', color='white')
        self.ax_main.set_ylabel('Acceleration (g)', color='white')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_facecolor('#0a0a0a')
        
        # Plot lines with vibrant colors
        self.line_x, = self.ax_main.plot([], [], '#ff4444', label='X-axis', linewidth=2, alpha=0.9)
        self.line_y, = self.ax_main.plot([], [], '#44ff44', label='Y-axis', linewidth=2, alpha=0.9)
        self.line_z, = self.ax_main.plot([], [], '#4444ff', label='Z-axis', linewidth=2, alpha=0.9)
        
        self.ax_main.legend(loc='upper right', fontsize=12)
        self.ax_main.set_ylim(-1.5, 1.5)  # Tighter range for better detail
        
        # Info panel
        self.ax_info.set_xlim(0, 10)
        self.ax_info.set_ylim(0, 10)
        self.ax_info.axis('off')
        self.ax_info.set_facecolor('#1a1a1a')
        
        # Text displays
        self.text_values = self.ax_info.text(0.02, 8, '', fontsize=14, color='white', 
                                           transform=self.ax_info.transAxes, ha='left')
        self.text_stats = self.ax_info.text(0.02, 6, '', fontsize=12, color='cyan',
                                          transform=self.ax_info.transAxes, ha='left')
        self.text_motion = self.ax_info.text(0.02, 4, '', fontsize=12, color='yellow',
                                           transform=self.ax_info.transAxes, ha='left')
        self.text_instructions = self.ax_info.text(0.02, 2, 'SPACE=Simulate | C=Real Data | F=File Monitor | R=Reset', 
                                                 fontsize=10, color='orange',
                                                 transform=self.ax_info.transAxes, ha='left')
        
        # Motion indicator
        self.motion_indicator = Rectangle((0.85, 0.7), 0.12, 0.25, 
                                        transform=self.ax_info.transAxes, 
                                        facecolor='green', alpha=0.7)
        self.ax_info.add_patch(self.motion_indicator)
        
        plt.tight_layout()
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == ' ':  # Spacebar to simulate data
            self.start_simulation()
        elif event.key == 'r':  # R to reset
            self.reset_data()
        elif event.key == 'f':  # F to start file monitoring
            self.start_file_monitoring()
        elif event.key == 'c':  # C to clear and prepare for real data
            self.prepare_for_real_data()
    
    def prepare_for_real_data(self):
        """Prepare for receiving real data from microcontroller"""
        self.reset_data()
        print("\n" + "="*60)
        print("🚁 READY FOR REAL MICROCONTROLLER DATA")
        print("="*60)
        print("📋 Method 1 - Copy/Paste:")
        print("   1. Copy console output from your microcontroller")
        print("   2. Paste it in this terminal window")
        print("   3. Watch real-time plotting!")
        print("")
        print("📁 Method 2 - File Monitoring:")
        print("   1. Press 'F' in the plot window")
        print("   2. Save microcontroller output to 'imu_data.txt'")
        print("   3. Automatic live plotting!")
        print("")
        print("🎯 Looking for lines like:")
        print("   📊 Rate: 150.0 Hz | Data: 0.146,-0.189,-0.055")
        print("="*60)
    
    def start_file_monitoring(self):
        """Start monitoring imu_data.txt file for real data"""
        self.file_monitoring = True
        print(f"\n📁 File monitoring started: {self.monitor_file}")
        print("💡 Save your microcontroller console output to 'imu_data.txt'")
        
        def monitor_file():
            import os
            while self.running and self.file_monitoring:
                try:
                    if os.path.exists(self.monitor_file):
                        with open(self.monitor_file, 'r') as f:
                            f.seek(self.file_position)
                            new_lines = f.readlines()
                            self.file_position = f.tell()
                            
                            for line in new_lines:
                                x, y, z = self.parse_console_line(line)
                                if x is not None:
                                    self.add_data_point(x, y, z)
                    
                    time.sleep(0.1)  # Check file every 100ms
                except Exception as e:
                    print(f"File monitoring error: {e}")
                    time.sleep(1)
        
        monitor_thread = threading.Thread(target=monitor_file, daemon=True)
        monitor_thread.start()
    
    def parse_console_line(self, line):
        """Parse console output line to extract IMU data - improved parsing"""
        # Multiple patterns to catch different formats
        patterns = [
            r'Data:\s*([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)',  # Rate: XXX Hz | Data: x,y,z
            r'([-+]?\d+\.?\d*),([-+]?\d+\.?\d*),([-+]?\d+\.?\d*)',  # Direct x,y,z format
            r'X[=:]?\s*([-+]?\d*\.?\d+).*Y[=:]?\s*([-+]?\d*\.?\d+).*Z[=:]?\s*([-+]?\d*\.?\d+)',  # X=x Y=y Z=z format
        ]
        
        for pattern in patterns:
            match = re.search(pattern, line)
            if match:
                try:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                    
                    # Sanity check - IMU values should be reasonable
                    if abs(x) < 10 and abs(y) < 10 and abs(z) < 10:
                        return x, y, z
                except (ValueError, IndexError):
                    continue
        return None, None, None
    
    def start_simulation(self):
        """Start data simulation based on your actual 190 Hz output"""
        print("🚀 Starting REALISTIC 190 Hz simulation based on your actual IMU data...")
        
        def simulate_data():
            base_time = time.time()
            sample_count = 0
            motion_phase = 0
            
            while self.running:
                current_time = time.time()
                dt = 1.0 / 190.0  # 190 Hz
                
                # Much more realistic IMU data based on your actual output
                # Base noise level (device at rest) - your actual values
                x = np.random.normal(0, 0.008)  # Increased base noise
                y = np.random.normal(0, 0.008)
                z = np.random.normal(0, 0.008)
                
                # Simulate device tilting (gravity vector changes)
                tilt_time = current_time * 0.3
                gravity_x = 0.1 * np.sin(tilt_time)
                gravity_y = 0.15 * np.cos(tilt_time * 0.7)
                gravity_z = 0.05 * np.sin(tilt_time * 1.2)
                
                x += gravity_x
                y += gravity_y  
                z += gravity_z
                
                # Sharp motion events (like your harsh movements)
                if np.random.random() < 0.005:  # 0.5% chance of sharp motion
                    # Big motion like your 0.146,-0.189,-0.055 example
                    motion_x = np.random.uniform(-0.3, 0.3)  # ±0.3g range
                    motion_y = np.random.uniform(-0.4, 0.4)  # ±0.4g range  
                    motion_z = np.random.uniform(-0.2, 0.2)  # ±0.2g range
                    
                    x += motion_x
                    y += motion_y
                    z += motion_z
                    motion_phase = 20  # Effects last for several samples
                
                # Motion decay (realistic IMU behavior)
                if motion_phase > 0:
                    decay_factor = motion_phase / 20.0
                    x += np.random.uniform(-0.1, 0.1) * decay_factor
                    y += np.random.uniform(-0.15, 0.15) * decay_factor
                    z += np.random.uniform(-0.08, 0.08) * decay_factor
                    motion_phase -= 1
                
                # Walking/vibration simulation
                if np.random.random() < 0.02:  # 2% chance of vibration
                    vibration_freq = current_time * 15  # Higher frequency
                    vibration_amp = 0.05
                    x += np.sin(vibration_freq) * vibration_amp
                    y += np.sin(vibration_freq * 1.3) * vibration_amp * 1.5
                    z += np.sin(vibration_freq * 0.8) * vibration_amp * 0.7
                
                # Occasional larger movements (picking up, setting down)
                if np.random.random() < 0.001:  # 0.1% chance of large motion
                    large_motion = np.random.uniform(0.5, 1.2)  # 0.5g to 1.2g
                    direction = np.random.choice([-1, 1])
                    axis = np.random.choice(['x', 'y', 'z'])
                    
                    if axis == 'x':
                        x += large_motion * direction
                    elif axis == 'y':
                        y += large_motion * direction
                    else:
                        z += large_motion * direction
                
                # Add some realistic sensor drift
                drift_time = current_time * 0.1
                x += 0.002 * np.sin(drift_time)
                y += 0.003 * np.cos(drift_time * 1.1)
                z += 0.001 * np.sin(drift_time * 0.9)
                
                self.add_data_point(x, y, z)
                
                # Maintain 190 Hz timing
                sample_count += 1
                expected_time = base_time + (sample_count * dt)
                sleep_time = expected_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        # Start simulation in background
        sim_thread = threading.Thread(target=simulate_data, daemon=True)
        sim_thread.start()
    
    def add_console_data(self, console_text):
        """Add data from console text"""
        lines = console_text.strip().split('\n')
        for line in lines:
            x, y, z = self.parse_console_line(line)
            if x is not None:
                self.add_data_point(x, y, z)
    
    def add_data_point(self, x, y, z):
        """Add new data point"""
        current_time = time.time()
        
        if self.start_time is None:
            self.start_time = current_time
            
        # Calculate rate
        if self.last_update:
            dt = current_time - self.last_update
            if dt > 0:
                self.current_rate = 1.0 / dt
                self.max_rate = max(self.max_rate, self.current_rate)
        
        self.last_update = current_time
        
        # Add data
        relative_time = current_time - self.start_time
        self.timestamps.append(relative_time)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        
        # Update current values
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.data_count += 1
        
        # Motion detection
        magnitude = (x*x + y*y + z*z)**0.5
        self.is_motion = magnitude > self.motion_threshold
    
    def reset_data(self):
        """Reset all data"""
        self.timestamps.clear()
        self.x_data.clear()
        self.y_data.clear()
        self.z_data.clear()
        self.data_count = 0
        self.start_time = None
        self.last_update = None
        self.current_rate = 0
        self.max_rate = 0
        print("📊 Data reset")
    
    def update_plot(self, frame):
        """Update the plot"""
        if len(self.timestamps) == 0:
            return self.line_x, self.line_y, self.line_z
        
        # Convert to arrays
        times = np.array(self.timestamps)
        x_vals = np.array(self.x_data)
        y_vals = np.array(self.y_data)
        z_vals = np.array(self.z_data)
        
        # Update plot lines
        self.line_x.set_data(times, x_vals)
        self.line_y.set_data(times, y_vals)
        self.line_z.set_data(times, z_vals)
        
        # Auto-scroll the x-axis (show last 10 seconds)
        if len(times) > 0:
            window_duration = 10
            latest_time = times[-1]
            self.ax_main.set_xlim(max(0, latest_time - window_duration), latest_time + 1)
        
        # Update text displays
        session_time = time.time() - self.start_time if self.start_time else 0
        avg_rate = self.data_count / session_time if session_time > 0 else 0
        
        self.text_values.set_text(f'Current: X={self.current_x:+.3f}g  Y={self.current_y:+.3f}g  Z={self.current_z:+.3f}g')
        
        self.text_stats.set_text(f'📊 Rate: {self.current_rate:.1f} Hz  |  Avg: {avg_rate:.1f} Hz  |  Max: {self.max_rate:.1f} Hz  |  Samples: {self.data_count}')
        
        # Motion indicator
        if self.is_motion:
            self.text_motion.set_text('🔥 MOTION DETECTED')
            self.motion_indicator.set_facecolor('red')
        else:
            self.text_motion.set_text('✅ Stable')
            self.motion_indicator.set_facecolor('green')
        
        return self.line_x, self.line_y, self.line_z
    
    def start_plotting(self):
        """Start the real-time plotting"""
        print("🚁 Real-Time IMU Plotter - Multiple Input Methods")
        print("=" * 60)
        print("📊 Window: Last 10 seconds | Update Rate: 60 FPS")
        print("🎯 Motion Detection: >0.1g threshold")
        print("")
        print("🎮 Controls (in plot window):")
        print("   SPACEBAR: Start realistic 190 Hz simulation")
        print("   C: Prepare for REAL microcontroller data")
        print("   F: Monitor imu_data.txt file") 
        print("   R: Reset data")
        print("   ESC: Close window")
        print("")
        print("📱 To see YOUR REAL DATA:")
        print("   1. Press 'C' for instructions")
        print("   2. Copy console data from microcontroller")
        print("   3. Paste here or save to imu_data.txt")
        print("")
        print("🛑 Close the plot window to stop")
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=16, 
                                    blit=False, cache_frame_data=False)
        
        # Start input thread for console data
        input_thread = threading.Thread(target=self.console_input_loop, daemon=True)
        input_thread.start()
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.file_monitoring = False
            print("\n📊 Final Statistics:")
            if self.start_time:
                total_time = time.time() - self.start_time
                print(f"   Session time: {total_time:.1f} seconds")
                print(f"   Total samples: {self.data_count}")
                if total_time > 0:
                    print(f"   Average rate: {self.data_count/total_time:.1f} Hz")
                print(f"   Maximum rate: {self.max_rate:.1f} Hz")
    
    def console_input_loop(self):
        """Loop to accept console input"""
        print("\n💡 You can paste console data here (or press SPACE in the plot window for simulation):")
        while self.running:
            try:
                line = input()
                if line.strip():
                    x, y, z = self.parse_console_line(line)
                    if x is not None:
                        self.add_data_point(x, y, z)
                    else:
                        # Try to parse multiple lines at once
                        self.add_console_data(line)
            except (EOFError, KeyboardInterrupt):
                break

def main():
    print("=" * 70)
    print("🚁 Real-Time IMU Data Plotter (190 Hz)")
    print("=" * 70)
    
    plotter = SerialIMUPlotter()
    plotter.start_plotting()

if __name__ == "__main__":
    main() 