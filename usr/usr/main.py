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
    # Create filters for each axis - Accelerometer
    filter_acc_x = LowPassFilter(FILTER_ALPHA)
    filter_acc_y = LowPassFilter(FILTER_ALPHA)
    filter_acc_z = LowPassFilter(FILTER_ALPHA)
    
    # Create filters for gyroscope
    filter_gyro_x = LowPassFilter(FILTER_ALPHA)
    filter_gyro_y = LowPassFilter(FILTER_ALPHA)
    filter_gyro_z = LowPassFilter(FILTER_ALPHA)
    
    # Create filters for magnetometer
    filter_mag_x = LowPassFilter(FILTER_ALPHA)
    filter_mag_y = LowPassFilter(FILTER_ALPHA)
    filter_mag_z = LowPassFilter(FILTER_ALPHA)

    try:
        while True:
            # Get all sensor data
            raw_accel = imu.get_accel()
            raw_gyro = imu.get_gyro()
            raw_mag = imu.get_mag()
            
            # Gravity compensation on Z-axis accelerometer
            compensated_z = raw_accel["z"] - 1.0
            
            # Apply low-pass filtering to accelerometer
            filtered_acc_x = filter_acc_x.apply(raw_accel["x"])
            filtered_acc_y = filter_acc_y.apply(raw_accel["y"])
            filtered_acc_z = filter_acc_z.apply(compensated_z)
            
            # Apply low-pass filtering to gyroscope
            filtered_gyro_x = filter_gyro_x.apply(raw_gyro["x"])
            filtered_gyro_y = filter_gyro_y.apply(raw_gyro["y"])
            filtered_gyro_z = filter_gyro_z.apply(raw_gyro["z"])
            
            # Apply low-pass filtering to magnetometer
            filtered_mag_x = filter_mag_x.apply(raw_mag["x"])
            filtered_mag_y = filter_mag_y.apply(raw_mag["y"])
            filtered_mag_z = filter_mag_z.apply(raw_mag["z"])
            
            # Format all sensor data for serial output
            data_line = "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}".format(
                filtered_acc_x, filtered_acc_y, filtered_acc_z,
                filtered_gyro_x, filtered_gyro_y, filtered_gyro_z,
                filtered_mag_x, filtered_mag_y, filtered_mag_z
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