import utime
from usr.config_manager import ConfigManager
from usr.imu_handler import IMUHandler

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

    FILTER_ALPHA = 0.2
    # Create filters for each axis
    filter_x = LowPassFilter(FILTER_ALPHA)
    filter_y = LowPassFilter(FILTER_ALPHA)
    filter_z = LowPassFilter(FILTER_ALPHA)



    try:
        while True:

            
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

        
            utime.sleep_ms(10)
            
    except KeyboardInterrupt:
        print("\n=== Shutting Down ===")
    finally:

        imu.stop()
        print("IMU and BLE beacon stopped")

if __name__ == "__main__":
    main()