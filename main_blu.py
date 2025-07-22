import utime
from usr.config_manager import ConfigManager
from usr.imu_handler import IMUHandler
from usr.anna_advertising_beacon import BLEAdvertisingBeacon

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

    # Initialize BLE Beacon
    print("=== Initializing BLE Beacon ===")
    ble_beacon = BLEAdvertisingBeacon(
        device_name="IMU_DRONE",
        use_extended_advertising=False,  # Use standard advertising for compatibility
        message_in_device_name=False     # Use manufacturer data for IMU readings
    )
    
    if not ble_beacon.initialize():
        print("Failed to initialize BLE beacon")
        return
    
    if not ble_beacon.start_advertising("IMU"):
        print("Failed to start BLE advertising")
        return

    SAMPLE_PERIOD_MS = 100  # IMU sample rate: 100ms (10 samples per second)
    FILTER_ALPHA = 0.2
    BLE_UPDATE_INTERVAL = 1  # Update BLE every sample (100ms) - CHANGED from 5
    
    # Create filters for each axis
    filter_x = LowPassFilter(FILTER_ALPHA)
    filter_y = LowPassFilter(FILTER_ALPHA)
    filter_z = LowPassFilter(FILTER_ALPHA)

    sample_count = 0
    last_ble_message = ""

    try:
        print("=== Starting IMU Data Collection & BLE Broadcasting ===")
        print("📊 IMU Rate: 10 Hz (100ms), BLE Rate: 10 Hz (100ms)")
        while True:
            # Check for BLE events (connections/disconnections)
            ble_beacon.check_events()
            
            raw_accel = imu.get_accel()
            
            # Gravity compensation on Z-axis
            compensated_z = raw_accel["z"] - 1.0
            
            # Apply low-pass filtering to all axes
            filtered_x = filter_x.apply(raw_accel["x"])
            filtered_y = filter_y.apply(raw_accel["y"])
            filtered_z = filter_z.apply(compensated_z)
            
            # Format filtered data for serial output
            data_line = "{:.3f},{:.3f},{:.3f}".format(
                filtered_x,
                filtered_y,
                filtered_z
            )
            
            print(data_line.strip())
            
            # Update BLE advertising every sample (100ms)
            sample_count += 1
            if sample_count >= BLE_UPDATE_INTERVAL:
                sample_count = 0
                
                # Create compact BLE message (limited to ~12 bytes for maximum speed)
                # Format: x.xx,y.xx,z.xx (each value 4 chars + comma = 14 chars max)
                ble_message = "{:.2f},{:.2f},{:.2f}".format(
                    filtered_x,
                    filtered_y, 
                    filtered_z
                )
                
                # Always update for maximum responsiveness (removed change check)
                if ble_beacon.update_message(ble_message):
                    pass  # Silent success for clean output
                else:
                    print("⚠️  BLE update failed")
                last_ble_message = ble_message
            
            utime.sleep_ms(SAMPLE_PERIOD_MS)
            
    except KeyboardInterrupt:
        print("\n=== Shutting Down ===")
    finally:
        # Clean shutdown
        ble_beacon.stop_advertising()
        ble_beacon.close()
        imu.stop()
        print("IMU and BLE beacon stopped")

if __name__ == "__main__":
    main()