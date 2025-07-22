import asyncio
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
from collections import deque
from datetime import datetime
from bleak import BleakScanner
import threading
import time

class RealtimeIMUPlotter:
    def __init__(self, target_device_names=["IMU_DRONE", "DTS", "IMU_DRONE_FAST"], window_size=500):
        self.target_device_names = target_device_names
        self.window_size = window_size  # Number of data points to display
        
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
        self.motion_threshold = 0.1  # g-force threshold for motion detection
        self.is_motion = False
        
        # Setup plot
        self.setup_plot()
        
        # BLE scanning
        self.ble_running = True
        self.ble_thread = None
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        # Create figure with dark theme
        plt.style.use('dark_background')
        self.fig, (self.ax_main, self.ax_info) = plt.subplots(2, 1, figsize=(12, 8), 
                                                              gridspec_kw={'height_ratios': [4, 1]})
        
        # Main plot setup
        self.ax_main.set_title('🚁 Real-Time IMU Data (190 Hz)', fontsize=16, color='white')
        self.ax_main.set_xlabel('Time (seconds)', color='white')
        self.ax_main.set_ylabel('Acceleration (g)', color='white')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_facecolor('#1a1a1a')
        
        # Plot lines
        self.line_x, = self.ax_main.plot([], [], 'r-', label='X-axis', linewidth=1.5, alpha=0.8)
        self.line_y, = self.ax_main.plot([], [], 'g-', label='Y-axis', linewidth=1.5, alpha=0.8)
        self.line_z, = self.ax_main.plot([], [], 'b-', label='Z-axis', linewidth=1.5, alpha=0.8)
        
        self.ax_main.legend(loc='upper right')
        self.ax_main.set_ylim(-2, 2)  # ±2g range
        
        # Info panel setup
        self.ax_info.set_xlim(0, 10)
        self.ax_info.set_ylim(0, 10)
        self.ax_info.axis('off')
        self.ax_info.set_facecolor('#2a2a2a')
        
        # Text displays for current values
        self.text_values = self.ax_info.text(0.5, 8, '', fontsize=12, color='white', 
                                           transform=self.ax_info.transAxes, ha='left')
        self.text_stats = self.ax_info.text(0.5, 6, '', fontsize=10, color='cyan',
                                          transform=self.ax_info.transAxes, ha='left')
        self.text_motion = self.ax_info.text(0.5, 4, '', fontsize=12, color='yellow',
                                           transform=self.ax_info.transAxes, ha='left')
        
        # Motion indicator
        self.motion_indicator = Rectangle((0.02, 0.7), 0.08, 0.2, 
                                        transform=self.ax_info.transAxes, 
                                        facecolor='green', alpha=0.7)
        self.ax_info.add_patch(self.motion_indicator)
        
        plt.tight_layout()
        
    def parse_service_data(self, service_data):
        """Parse service data to extract IMU values"""
        try:
            for service_uuid, data in service_data.items():
                try:
                    imu_string = data.decode('utf-8').strip()
                    if imu_string and ',' in imu_string:
                        return imu_string
                except UnicodeDecodeError:
                    continue
        except Exception:
            pass
        return None
    
    def parse_manufacturer_data(self, manufacturer_data):
        """Parse manufacturer data to extract IMU values"""
        try:
            for company_id, data in manufacturer_data.items():
                try:
                    imu_string = data.decode('utf-8').strip()
                    if imu_string and ',' in imu_string:
                        return imu_string
                except UnicodeDecodeError:
                    continue
        except Exception:
            pass
        return None
    
    def parse_imu_values(self, imu_string):
        """Parse IMU string into x, y, z values"""
        try:
            parts = imu_string.split(',')
            if len(parts) == 3:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                return x, y, z
        except (ValueError, IndexError):
            pass
        return None, None, None
    
    def detection_callback(self, device, advertisement_data):
        """BLE detection callback"""
        # Filter by device name
        device_match = False
        if device.name:
            for target_name in self.target_device_names:
                if target_name in device.name:
                    device_match = True
                    break
        
        if not device_match:
            return
        
        # Try to get IMU data
        imu_string = None
        
        # Check Service Data first
        if advertisement_data.service_data:
            imu_string = self.parse_service_data(advertisement_data.service_data)
        
        # Fall back to Manufacturer Data
        if not imu_string and advertisement_data.manufacturer_data:
            imu_string = self.parse_manufacturer_data(advertisement_data.manufacturer_data)
        
        if imu_string:
            x, y, z = self.parse_imu_values(imu_string)
            if x is not None:
                self.add_data_point(x, y, z)
    
    def add_data_point(self, x, y, z):
        """Add new data point to the plot"""
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
    
    def update_plot(self, frame):
        """Update the plot (called by animation)"""
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
        
        # Auto-scroll the x-axis
        if len(times) > 0:
            window_duration = 10  # Show last 10 seconds
            latest_time = times[-1]
            self.ax_main.set_xlim(max(0, latest_time - window_duration), latest_time + 1)
        
        # Update text displays
        session_time = time.time() - self.start_time if self.start_time else 0
        avg_rate = self.data_count / session_time if session_time > 0 else 0
        
        self.text_values.set_text(f'Current: X={self.current_x:+.3f}g  Y={self.current_y:+.3f}g  Z={self.current_z:+.3f}g')
        
        self.text_stats.set_text(f'Rate: {self.current_rate:.1f} Hz  |  Avg: {avg_rate:.1f} Hz  |  Max: {self.max_rate:.1f} Hz  |  Samples: {self.data_count}')
        
        # Motion indicator
        if self.is_motion:
            self.text_motion.set_text('🔥 MOTION DETECTED')
            self.motion_indicator.set_facecolor('red')
        else:
            self.text_motion.set_text('✅ Stable')
            self.motion_indicator.set_facecolor('green')
        
        return self.line_x, self.line_y, self.line_z
    
    async def ble_scanner(self):
        """BLE scanning loop"""
        print("🔍 Starting BLE scanner for real-time plotting...")
        print(f"📡 Looking for: {', '.join(self.target_device_names)}")
        
        while self.ble_running:
            try:
                scanner = BleakScanner(detection_callback=self.detection_callback)
                await scanner.start()
                
                while self.ble_running:
                    await asyncio.sleep(0.1)
                    
                await scanner.stop()
            except Exception as e:
                print(f"BLE Error: {e}")
                await asyncio.sleep(1)
    
    def start_ble_thread(self):
        """Start BLE scanning in a separate thread"""
        def run_ble():
            asyncio.run(self.ble_scanner())
        
        self.ble_thread = threading.Thread(target=run_ble, daemon=True)
        self.ble_thread.start()
    
    def start_plotting(self):
        """Start the real-time plotting"""
        print("🚀 Starting Real-Time IMU Plotter")
        print("📊 Window: Last 10 seconds | Update Rate: 60 FPS")
        print("🎯 Motion Detection: >0.1g threshold")
        print("🛑 Close the plot window to stop")
        
        # Start BLE scanning
        self.start_ble_thread()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=16, blit=False)  # ~60 FPS
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.ble_running = False
            print("\n📊 Final Statistics:")
            if self.start_time:
                total_time = time.time() - self.start_time
                print(f"   Session time: {total_time:.1f} seconds")
                print(f"   Total samples: {self.data_count}")
                print(f"   Average rate: {self.data_count/total_time:.1f} Hz")
                print(f"   Maximum rate: {self.max_rate:.1f} Hz")

def main():
    print("=" * 60)
    print("🚁 Real-Time IMU Data Plotter (190 Hz)")
    print("=" * 60)
    
    plotter = RealtimeIMUPlotter()
    plotter.start_plotting()

if __name__ == "__main__":
    main() 