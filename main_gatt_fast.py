import utime
from usr.config_manager import ConfigManager
from usr.imu_handler import IMUHandler
from usr.anna_advertising_beacon import BLEGATTServer

# Low-pass filter implementation
class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha  # Smoothing factor (0.0 - 1.0)
        self.prev_value = 0.0
        
    def apply(self, value):
        filtered = self.alpha * value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered
        return filtered

def main():
    # Initialize IMU
    try:
        cfg_mgr = ConfigManager()
    except Exception:
        cfg_mgr = ConfigManager("config.json")

    imu = IMUHandler(cfg_mgr)
    if not imu.start():
        print("Failed to start IMU handler")
        return

    # Initialize High-Speed BLE GATT Server
    print("=== Initializing High-Speed BLE GATT Server ===")
    gatt_server = BLEGATTServer(device_name="IMU_DRONE_FAST")
    
    if not gatt_server.initialize():
        print("Failed to initialize GATT server")
        return
    
    if not gatt_server.start_server():
        print("Failed to start GATT server")
        return

    # High-speed parameters
    SAMPLE_PERIOD_MS = 10   # 100 Hz IMU sampling (10x faster)
    FILTER_ALPHA = 0.2
    GATT_UPDATE_INTERVAL = 1  # Send via GATT every sample (100 Hz)
    
    # Create filters for each axis
    filter_x = LowPassFilter(FILTER_ALPHA)
    filter_y = LowPassFilter(FILTER_ALPHA)
    filter_z = LowPassFilter(FILTER_ALPHA)

    sample_count = 0
    total_samples = 0
    start_time = utime.ticks_ms()

    try:
        print("=== Starting High-Speed IMU Data Streaming ===")
        print("📊 IMU Rate: 100 Hz (10ms), GATT Rate: 100 Hz")
        print("🚀 Waiting for BLE connection to start streaming...")
        
        while True:
            loop_start = utime.ticks_ms()
            
            # Check for connection events
            gatt_server.check_events()
            
            raw_accel = imu.get_accel()
            
            # Gravity compensation on Z-axis
            compensated_z = raw_accel["z"] - 1.0
            
            # Apply low-pass filtering to all axes
            filtered_x = filter_x.apply(raw_accel["x"])
            filtered_y = filter_y.apply(raw_accel["y"])
            filtered_z = filter_z.apply(compensated_z)
            
            # High-precision IMU data format for GATT (more room than advertising)
            data_line = "{:.3f},{:.3f},{:.3f}".format(
                filtered_x,
                filtered_y,
                filtered_z
            )
            
            # Print to console (less frequent to avoid spam)
            total_samples += 1
            if total_samples % 50 == 0:  # Print every 50 samples (0.5 seconds)
                elapsed = utime.ticks_diff(utime.ticks_ms(), start_time)
                actual_rate = (total_samples * 1000) / elapsed if elapsed > 0 else 0
                print(f"📊 Rate: {actual_rate:.1f} Hz | Data: {data_line}")
            
            # Send via GATT notifications (much faster than advertising)
            sample_count += 1
            if sample_count >= GATT_UPDATE_INTERVAL:
                sample_count = 0
                
                # Enhanced data format with more precision and metadata
                gatt_message = f"{filtered_x:.3f},{filtered_y:.3f},{filtered_z:.3f}"
                
                if gatt_server.send_imu_data(gatt_message):
                    # Success - no need to print every success for high-speed operation
                    pass
                else:
                    # Only print if not connected (to avoid spam)
                    if not gatt_server.is_connected and total_samples % 100 == 0:
                        print("🔍 Waiting for BLE connection...")
            
            # Precise timing control for consistent 100 Hz
            loop_time = utime.ticks_diff(utime.ticks_ms(), loop_start)
            remaining = SAMPLE_PERIOD_MS - loop_time
            if remaining > 0:
                utime.sleep_ms(remaining)
            
    except KeyboardInterrupt:
        print("\n=== Shutting Down ===")
    finally:
        # Clean shutdown
        gatt_server.close()
        imu.stop()
        
        # Print final statistics
        total_time = utime.ticks_diff(utime.ticks_ms(), start_time) / 1000.0
        if total_time > 0:
            final_rate = total_samples / total_time
            print(f"📊 Final Statistics:")
            print(f"   Total samples: {total_samples}")
            print(f"   Session time: {total_time:.1f} seconds")
            print(f"   Average rate: {final_rate:.1f} Hz")
        print("High-speed GATT server stopped")

if __name__ == "__main__":
    main() 