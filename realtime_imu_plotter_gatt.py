import asyncio
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import numpy as np
from collections import deque
from datetime import datetime
from bleak import BleakClient, BleakScanner
import threading
import time

class RealtimeIMUGATTPlotter:
    def __init__(self, target_device_name="IMU_DRONE_FAST", window_size=1000):
        self.target_device_name = target_device_name
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
        
        # GATT connection
        self.client = None
        self.device = None
        self.is_connected = False
        self.gatt_running = True
        self.gatt_thread = None
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        # Create figure with dark theme
        plt.style.use('dark_background')
        self.fig, (self.ax_main, self.ax_info) = plt.subplots(2, 1, figsize=(14, 10), 
                                                              gridspec_kw={'height_ratios': [4, 1]})
        
        # Main plot setup
        self.ax_main.set_title('🚀 Real-Time IMU Data - GATT Notifications (190 Hz)', fontsize=16, color='white')
        self.ax_main.set_xlabel('Time (seconds)', color='white')
        self.ax_main.set_ylabel('Acceleration (g)', color='white')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_facecolor('#1a1a1a')
        
        # Plot lines with improved styling
        self.line_x, = self.ax_main.plot([], [], 'r-', label='X-axis', linewidth=2, alpha=0.9)
        self.line_y, = self.ax_main.plot([], [], 'g-', label='Y-axis', linewidth=2, alpha=0.9)
        self.line_z, = self.ax_main.plot([], [], 'b-', label='Z-axis', linewidth=2, alpha=0.9)
        
        self.ax_main.legend(loc='upper right', fontsize=12)
        self.ax_main.set_ylim(-2, 2)  # ±2g range
        
        # Info panel setup
        self.ax_info.set_xlim(0, 10)
        self.ax_info.set_ylim(0, 10)
        self.ax_info.axis('off')
        self.ax_info.set_facecolor('#2a2a2a')
        
        # Text displays for current values
        self.text_values = self.ax_info.text(0.02, 8, '', fontsize=14, color='white', 
                                           transform=self.ax_info.transAxes, ha='left')
        self.text_stats = self.ax_info.text(0.02, 6, '', fontsize=12, color='cyan',
                                          transform=self.ax_info.transAxes, ha='left')
        self.text_motion = self.ax_info.text(0.02, 4, '', fontsize=12, color='yellow',
                                           transform=self.ax_info.transAxes, ha='left')
        self.text_connection = self.ax_info.text(0.02, 2, '', fontsize=12, color='orange',
                                               transform=self.ax_info.transAxes, ha='left')
        
        # Motion indicator
        self.motion_indicator = Rectangle((0.85, 0.7), 0.12, 0.25, 
                                        transform=self.ax_info.transAxes, 
                                        facecolor='green', alpha=0.7)
        self.ax_info.add_patch(self.motion_indicator)
        
        plt.tight_layout()
        
    async def discover_device(self, timeout=10):
        """Discover the target GATT server device"""
        print(f"🔍 Scanning for '{self.target_device_name}'...")
        
        devices = await BleakScanner.discover(timeout=timeout)
        
        for device in devices:
            if device.name and self.target_device_name in device.name:
                print(f"✅ Found device: {device.name} ({device.address})")
                self.device = device
                return True
                
        print(f"❌ Device '{self.target_device_name}' not found")
        return False
    
    def notification_callback(self, sender, data):
        """Handle incoming GATT notifications (high-speed data)"""
        try:
            # Decode the notification data
            message = data.decode('utf-8').strip()
            
            # Parse enhanced data format from GATT server
            if message.startswith("IMU,"):
                # Format: "IMU,timestamp,x.xxx,y.xxx,z.xxx,length"
                parts = message.split(',')
                if len(parts) >= 5:
                    try:
                        server_timestamp = float(parts[1])
                        x = float(parts[2])
                        y = float(parts[3])
                        z = float(parts[4])
                        self.add_data_point(x, y, z, server_timestamp)
                    except (ValueError, IndexError):
                        pass
            else:
                # Simple format: "x.xxx,y.xxx,z.xxx"
                parts = message.split(',')
                if len(parts) == 3:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        z = float(parts[2])
                        self.add_data_point(x, y, z)
                    except (ValueError, IndexError):
                        pass
                        
        except Exception as e:
            print(f"Notification decode error: {e}")
    
    def add_data_point(self, x, y, z, server_timestamp=None):
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
    
    async def connect_and_stream(self):
        """Connect to GATT server and start receiving notifications"""
        if not self.device:
            return False
            
        try:
            print(f"🔗 Connecting to {self.device.name}...")
            
            self.client = BleakClient(self.device.address)
            await self.client.connect()
            
            if not self.client.is_connected:
                print("❌ Failed to connect")
                return False
                
            self.is_connected = True
            print(f"✅ Connected to {self.device.name}")
            
            # Discover services
            print("🔍 Discovering services...")
            try:
                services = await self.client.get_services()
            except AttributeError:
                services = self.client.services
            
            # Find any notifiable characteristic
            target_char = None
            for service in services:
                for char in service.characteristics:
                    if ("notify" in char.properties or "indicate" in char.properties):
                        target_char = char
                        print(f"📡 Found notifiable characteristic: {char.uuid}")
                        break
                if target_char:
                    break
            
            if not target_char:
                print("❌ No notifiable characteristic found")
                print("💡 Using basic connection mode...")
                # Keep connection alive for demonstration
                while self.gatt_running and self.client.is_connected:
                    await asyncio.sleep(1)
                return True
            
            # Enable notifications
            print(f"📡 Enabling notifications on {target_char.uuid}...")
            try:
                await self.client.start_notify(target_char.uuid, self.notification_callback)
                print("🚀 GATT notifications active! Receiving 190 Hz data...")
                
                # Keep connection alive and receive notifications
                while self.gatt_running and self.client.is_connected:
                    await asyncio.sleep(0.01)
                    
                # Stop notifications
                await self.client.stop_notify(target_char.uuid)
                
            except Exception as e:
                print(f"⚠️  Notification setup failed: {e}")
                # Keep basic connection alive
                while self.gatt_running and self.client.is_connected:
                    await asyncio.sleep(1)
                
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False
        finally:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                print("📡 Disconnected from GATT server")
            self.is_connected = False
        
        return True
    
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
        
        # Auto-scroll the x-axis (show last 15 seconds for high-speed data)
        if len(times) > 0:
            window_duration = 15  # Show last 15 seconds
            latest_time = times[-1]
            self.ax_main.set_xlim(max(0, latest_time - window_duration), latest_time + 1)
        
        # Update text displays
        session_time = time.time() - self.start_time if self.start_time else 0
        avg_rate = self.data_count / session_time if session_time > 0 else 0
        
        self.text_values.set_text(f'Current: X={self.current_x:+.3f}g  Y={self.current_y:+.3f}g  Z={self.current_z:+.3f}g')
        
        self.text_stats.set_text(f'📊 Rate: {self.current_rate:.1f} Hz  |  Avg: {avg_rate:.1f} Hz  |  Max: {self.max_rate:.1f} Hz  |  Samples: {self.data_count}')
        
        # Connection status
        connection_status = "🔗 Connected (GATT)" if self.is_connected else "🔍 Connecting..."
        self.text_connection.set_text(connection_status)
        
        # Motion indicator
        if self.is_motion:
            self.text_motion.set_text('🔥 MOTION DETECTED')
            self.motion_indicator.set_facecolor('red')
        else:
            self.text_motion.set_text('✅ Stable')
            self.motion_indicator.set_facecolor('green')
        
        return self.line_x, self.line_y, self.line_z
    
    def start_gatt_thread(self):
        """Start GATT connection in a separate thread"""
        async def run_gatt():
            # Discover and connect
            if await self.discover_device(timeout=15):
                await self.connect_and_stream()
            else:
                print("💡 Make sure your microcontroller is running main.py (GATT server)")
        
        def gatt_wrapper():
            asyncio.run(run_gatt())
        
        self.gatt_thread = threading.Thread(target=gatt_wrapper, daemon=True)
        self.gatt_thread.start()
    
    def start_plotting(self):
        """Start the real-time plotting"""
        print("🚀 Starting Real-Time GATT IMU Plotter")
        print("📊 Window: Last 15 seconds | Update Rate: 60 FPS")
        print("🎯 Motion Detection: >0.1g threshold") 
        print("⚡ Expecting 190 Hz GATT notifications")
        print("🛑 Close the plot window to stop")
        
        # Start GATT connection
        self.start_gatt_thread()
        
        # Start animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=16, blit=False, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.gatt_running = False
            print("\n📊 Final Statistics:")
            if self.start_time:
                total_time = time.time() - self.start_time
                print(f"   Session time: {total_time:.1f} seconds")
                print(f"   Total samples: {self.data_count}")
                if total_time > 0:
                    print(f"   Average rate: {self.data_count/total_time:.1f} Hz")
                print(f"   Maximum rate: {self.max_rate:.1f} Hz")

def main():
    print("=" * 70)
    print("🚀 Real-Time IMU GATT Plotter (190 Hz)")
    print("=" * 70)
    
    plotter = RealtimeIMUGATTPlotter()
    plotter.start_plotting()

if __name__ == "__main__":
    main() 