import asyncio
import struct
from bleak import BleakScanner
from datetime import datetime

class IMUBLEListener:
    def __init__(self, target_device_names=["IMU_DRONE", "DTS"], debug=False):
        self.target_device_names = target_device_names  # Support both full and short names
        self.last_data = None
        self.data_count = 0
        self.last_update_time = None
        self.debug = debug  # Debug mode to show raw data
        self.last_message = None  # Track last message to filter duplicates
        self.unique_count = 0     # Count unique messages only
        self.start_time = None    # Track session start time
        
    def parse_manufacturer_data(self, manufacturer_data):
        """Parse manufacturer data to extract IMU values - fixed for proper company ID handling"""
        try:
            # manufacturer_data is a dict: {company_id: bytes_data}
            for company_id, data in manufacturer_data.items():
                if self.debug:
                    print(f"    DEBUG: Company ID: 0x{company_id:04X}, Data: {data.hex()}, Bytes: {len(data)}")
                
                try:
                    # For our beacon using company ID 0xFFFF, the data structure is:
                    # The company_id is the key, and data contains our IMU string directly
                    # No need to skip bytes - the company ID is already separated
                    imu_string = data.decode('utf-8').strip()
                    if self.debug:
                        print(f"    DEBUG: Decoded string: '{imu_string}'")
                    if imu_string and ',' in imu_string:  # Basic validation
                        return imu_string
                except UnicodeDecodeError:
                    if self.debug:
                        print(f"    DEBUG: UTF-8 decode failed, trying alternatives...")
                    # Try alternative parsing for malformed data
                    try:
                        # Sometimes data might have extra bytes, try different positions
                        for offset in [0, 1, 2]:
                            if len(data) > offset:
                                test_string = data[offset:].decode('utf-8').strip()
                                if self.debug:
                                    print(f"    DEBUG: Offset {offset}: '{test_string}'")
                                if ',' in test_string and len(test_string.split(',')) == 3:
                                    return test_string
                    except:
                        pass
                    continue
                    
        except Exception as e:
            if self.debug:
                print(f"    DEBUG: Parse error: {e}")
        
        return None
    
    def parse_imu_values(self, imu_string):
        """Parse IMU string into x, y, z values - optimized"""
        try:
            # Expected format: "x.xx,y.xx,z.xx"
            parts = imu_string.split(',')
            if len(parts) == 3:
                x = float(parts[0])
                y = float(parts[1]) 
                z = float(parts[2])
                return x, y, z
        except (ValueError, IndexError):
            pass
        return None, None, None
    
    def parse_service_data(self, service_data):
        """Parse service data to extract IMU values - for improved bandwidth"""
        try:
            # service_data is a dict: {service_uuid: bytes_data}
            for service_uuid, data in service_data.items():
                if self.debug:
                    print(f"    DEBUG: Service UUID: {service_uuid}, Data: {data.hex()}, Bytes: {len(data)}")
                
                try:
                    # Service data format: our IMU string directly (no extra headers)
                    imu_string = data.decode('utf-8').strip()
                    if self.debug:
                        print(f"    DEBUG: Service decoded: '{imu_string}'")
                    if imu_string and ',' in imu_string:
                        return imu_string
                except UnicodeDecodeError:
                    if self.debug:
                        print(f"    DEBUG: Service UTF-8 decode failed")
                    continue
                    
        except Exception as e:
            if self.debug:
                print(f"    DEBUG: Service parse error: {e}")
        
        return None

    def detection_callback(self, device, advertisement_data):
        """Callback function for BLE advertisements - optimized for high frequency"""
        
        # Filter by device name (check both possible names)
        device_name_match = False
        if device.name:
            for target_name in self.target_device_names:
                if target_name in device.name:
                    device_name_match = True
                    break
        
        if not device_name_match:
            return
            
        current_time = datetime.now()
        if self.start_time is None:
            self.start_time = current_time
            
        timestamp = current_time.strftime("%H:%M:%S.%f")[:-3]
        
        # Try Service Data first (better bandwidth), then fall back to Manufacturer Data
        imu_string = None
        data_source = ""
        
        # Check Service Data first (improved bandwidth)
        if advertisement_data.service_data:
            imu_string = self.parse_service_data(advertisement_data.service_data)
            if imu_string:
                data_source = "Service"
        
        # Fall back to Manufacturer Data if Service Data not found
        if not imu_string and advertisement_data.manufacturer_data:
            imu_string = self.parse_manufacturer_data(advertisement_data.manufacturer_data)
            if imu_string:
                data_source = "Manufacturer"
        
        if imu_string:
            # Filter duplicates - only process if message changed
            if imu_string == self.last_message:
                return  # Skip duplicate
                
            self.last_message = imu_string
            
            # Try to parse as IMU values
            x, y, z = self.parse_imu_values(imu_string)
            if x is not None:
                self.data_count += 1
                self.unique_count += 1
                
                # Calculate accurate rate based on unique messages
                rate_info = ""
                if self.last_update_time:
                    time_diff = (current_time - self.last_update_time).total_seconds()
                    if time_diff > 0:
                        instant_hz = 1.0 / time_diff
                        rate_info = f" ({instant_hz:.1f}Hz)"
                
                # Calculate average rate since start
                if self.start_time:
                    total_time = (current_time - self.start_time).total_seconds()
                    if total_time > 0:
                        avg_hz = self.unique_count / total_time
                        rate_info += f" Avg:{avg_hz:.1f}Hz"
                
                self.last_update_time = current_time
                
                # Clean, compact output with better rate info
                print(f"[{timestamp}]{rate_info} #{self.unique_count:4d} | X:{x:+6.2f} Y:{y:+6.2f} Z:{z:+6.2f} | RSSI:{advertisement_data.rssi:3d}dBm")
                
                self.last_data = (x, y, z, timestamp)
            else:
                print(f"[{timestamp}] ⚠️  Parse error: '{imu_string}' ({data_source})")
        else:
            if self.debug:
                print(f"[{timestamp}] ❌ No IMU data found")
    
    async def start_scanning(self, duration=None):
        """Start scanning for BLE advertisements"""
        target_names_str = "'" + "' or '".join(self.target_device_names) + "'"
        print(f"🔍 Scanning for: {target_names_str}")
        print("📡 Listening for consistent 10 Hz IMU data (duplicates filtered)...")
        print("🛑 Press Ctrl+C to stop")
        print("")
        print("Time         Rate Info        #    |   X      Y      Z   | Signal")
        print("-" * 75)
        
        try:
            if duration:
                await BleakScanner.discover(timeout=duration, detection_callback=self.detection_callback)
            else:
                # Continuous scanning with optimized settings for high-frequency data
                scanner = BleakScanner(detection_callback=self.detection_callback)
                await scanner.start()
                
                try:
                    while True:
                        await asyncio.sleep(0.05)  # Very fast polling for ultra-responsive scanning
                except KeyboardInterrupt:
                    print("\n🛑 Stopping scan...")
                finally:
                    await scanner.stop()
                    
        except Exception as e:
            print(f"❌ Scanning error: {e}")
    
    def print_summary(self):
        """Print summary of received data"""
        print(f"\n📊 Summary:")
        print(f"   Total packets received: {self.data_count}")
        print(f"   Unique IMU readings: {self.unique_count}")
        if self.unique_count > 0 and self.start_time:
            session_time = (datetime.now() - self.start_time).total_seconds()
            avg_rate = self.unique_count / session_time if session_time > 0 else 0
            print(f"   Session duration: {session_time:.1f} seconds")
            print(f"   Average data rate: {avg_rate:.1f} Hz")
            efficiency = (self.unique_count / self.data_count * 100) if self.data_count > 0 else 0
            print(f"   Data efficiency: {efficiency:.1f}% (unique/total)")
        if self.last_data:
            x, y, z, timestamp = self.last_data
            print(f"   Last reading: X={x:+.2f}, Y={y:+.2f}, Z={z:+.2f} at {timestamp}")

async def main():
    """Main function"""
    print("=" * 60)
    print("🚁 IMU Drone BLE Listener (Optimized for 10 Hz)")
    print("=" * 60)
    
    # Create listener for both device names (full and short) - debug disabled
    listener = IMUBLEListener(target_device_names=["IMU_DRONE", "DTS"], debug=False)
    
    try:
        await listener.start_scanning()
    except KeyboardInterrupt:
        print("\n👋 Scan stopped by user")
    finally:
        listener.print_summary()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")