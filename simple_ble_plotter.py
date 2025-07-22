import asyncio
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import time
from bleak import BleakScanner, BleakClient
import threading

class SimpleBLEPlotter:
    def __init__(self):
        self.x_data = deque(maxlen=1000)
        self.y_data = deque(maxlen=1000)
        self.z_data = deque(maxlen=1000)
        self.timestamps = deque(maxlen=1000)
        
        self.client = None
        self.running = True
        self.start_time = time.time()
        self.data_count = 0
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title('IMU Data Stream')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Acceleration (g)')
        self.ax.grid(True)
        
        self.line_x, = self.ax.plot([], [], 'r-', label='X')
        self.line_y, = self.ax.plot([], [], 'g-', label='Y')
        self.line_z, = self.ax.plot([], [], 'b-', label='Z')
        self.ax.legend()
        self.ax.set_ylim(-2, 2)

    async def find_device(self):
        """Find the IMU device"""
        print("Scanning for IMU_DRONE_FAST...")
        devices = await BleakScanner.discover(timeout=10)
        
        for device in devices:
            if device.name and "IMU_DRONE_FAST" in device.name:
                print(f"Found: {device.name}")
                return device
        print("Device not found")
        return None

    def data_callback(self, sender, data):
        """Handle incoming BLE data"""
        try:
            message = data.decode('utf-8').strip()
            parts = message.split(',')
            
            if len(parts) == 3:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                
                current_time = time.time() - self.start_time
                self.timestamps.append(current_time)
                self.x_data.append(x)
                self.y_data.append(y)
                self.z_data.append(z)
                
                self.data_count += 1
                if self.data_count % 100 == 0:  # Print every 100 samples
                    print(f"Received {self.data_count} samples - Latest: {x:.3f}, {y:.3f}, {z:.3f}")
                
        except Exception as e:
            pass  # Ignore decode errors

    async def connect_and_listen(self):
        """Connect to device and start listening"""
        device = await self.find_device()
        if not device:
            return

        try:
            self.client = BleakClient(device.address)
            await self.client.connect()
            print("Connected")

            # Find notification characteristic
            try:
                services = await self.client.get_services()
            except AttributeError:
                services = self.client.services
                
            char = None
            for service in services:
                for c in service.characteristics:
                    if "notify" in c.properties:
                        char = c
                        break
                if char:
                    break

            if char:
                await self.client.start_notify(char.uuid, self.data_callback)
                print("Receiving data...")
                
                while self.running:
                    await asyncio.sleep(0.1)
                    
                await self.client.stop_notify(char.uuid)
            
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self.client:
                await self.client.disconnect()

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
                self.ax.set_xlim(max(0, times[-1] - 10), times[-1] + 1)
        
        return self.line_x, self.line_y, self.line_z

    def start(self):
        """Start the plotter"""
        print("Starting BLE plotter...")
        
        # Start BLE in background
        def ble_thread():
            asyncio.run(self.connect_and_listen())
        
        threading.Thread(target=ble_thread, daemon=True).start()
        
        # Start plot animation
        ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False

if __name__ == "__main__":
    plotter = SimpleBLEPlotter()
    plotter.start() 