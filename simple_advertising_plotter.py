import asyncio
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import time
from bleak import BleakScanner
import threading

class SimpleAdvertisingPlotter:
    def __init__(self):
        self.x_data = deque(maxlen=500)
        self.y_data = deque(maxlen=500)
        self.z_data = deque(maxlen=500)
        self.timestamps = deque(maxlen=500)
        
        self.running = True
        self.start_time = time.time()
        self.data_count = 0
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title('IMU Data from BLE Advertising')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Acceleration (g)')
        self.ax.grid(True)
        
        self.line_x, = self.ax.plot([], [], 'r-', label='X')
        self.line_y, = self.ax.plot([], [], 'g-', label='Y')
        self.line_z, = self.ax.plot([], [], 'b-', label='Z')
        self.ax.legend()
        self.ax.set_ylim(-2, 2)

    def parse_advertising_data(self, device, advertisement_data):
        """Parse BLE advertising data for IMU values"""
        if not device.name or "IMU_DRONE" not in device.name:
            return
            
        try:
            # Check manufacturer data
            if advertisement_data.manufacturer_data:
                for company_id, data in advertisement_data.manufacturer_data.items():
                    if len(data) >= 12:  # 3 floats = 12 bytes
                        # Parse as 3 x 32-bit floats
                        x = int.from_bytes(data[0:4], 'little', signed=True) / 1000.0
                        y = int.from_bytes(data[4:8], 'little', signed=True) / 1000.0
                        z = int.from_bytes(data[8:12], 'little', signed=True) / 1000.0
                        self.add_data_point(x, y, z)
                        return
            
            # Check service data
            if advertisement_data.service_data:
                for service_uuid, data in advertisement_data.service_data.items():
                    if len(data) >= 12:
                        x = int.from_bytes(data[0:4], 'little', signed=True) / 1000.0
                        y = int.from_bytes(data[4:8], 'little', signed=True) / 1000.0
                        z = int.from_bytes(data[8:12], 'little', signed=True) / 1000.0
                        self.add_data_point(x, y, z)
                        return
                        
        except Exception as e:
            pass

    def add_data_point(self, x, y, z):
        """Add new data point"""
        current_time = time.time() - self.start_time
        self.timestamps.append(current_time)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        
        self.data_count += 1
        if self.data_count % 50 == 0:
            print(f"Data: {x:.3f}, {y:.3f}, {z:.3f}")

    async def scan_for_data(self):
        """Continuously scan for advertising data"""
        print("Scanning for IMU advertising data...")
        
        while self.running:
            try:
                # Use discover method instead of callback registration
                devices = await BleakScanner.discover(timeout=1.0, return_adv=True)
                
                for device, advertisement_data in devices.values():
                    self.parse_advertising_data(device, advertisement_data)
                
            except Exception as e:
                print(f"Scan error: {e}")
                await asyncio.sleep(1)

    def update_plot(self, frame):
        """Update the plot"""
        if len(self.timestamps) > 0:
            times = np.array(self.timestamps)
            x_vals = np.array(self.x_data)
            y_vals = np.array(self.y_data)
            z_vals = np.array(self.z_data)
            
            self.line_x.set_data(times, x_vals)
            self.line_y.set_data(times, y_vals)
            self.line_z.set_data(times, z_vals)
            
            # Auto-scroll
            if len(times) > 0:
                self.ax.set_xlim(max(0, times[-1] - 15), times[-1] + 1)
        
        return self.line_x, self.line_y, self.line_z

    def start(self):
        """Start the plotter"""
        print("Starting advertising plotter...")
        
        # Start BLE scanning in background
        def ble_thread():
            asyncio.run(self.scan_for_data())
        
        threading.Thread(target=ble_thread, daemon=True).start()
        
        # Start plot animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            print(f"\nReceived {self.data_count} data points")

if __name__ == "__main__":
    plotter = SimpleAdvertisingPlotter()
    plotter.start() 