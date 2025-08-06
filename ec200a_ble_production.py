"""
Production-Ready EC200A BLE Configuration
Ultra-optimized for maximum speed and reliability
"""

from machine import UART
import time

class EC200ABLEProduction:
    """Production-ready BLE configuration for EC200A/ANNA-B4"""
    
    def __init__(self, device_name="ANNA"):
        self.device_name = device_name
        self.uart = None
        
    def send_at_fast(self, command, timeout=1):
        """Ultra-fast AT command with minimal overhead"""
        if not self.uart:
            return "ERROR"
            
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.01)  # Minimal delay
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response:
                        break
            time.sleep(0.002)  # Ultra-fast polling
        
        return response.decode().strip()
    
    def configure_ble_ultra_fast(self):
        """Ultra-fast BLE configuration - production ready"""
        print("🚀 Ultra-Fast BLE Configuration")
        
        try:
            # Initialize UART
            self.uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
            
            # Quick AT test
            if "OK" not in self.send_at_fast("AT"):
                print("❌ Module not responding")
                return False
            
            # Disable echo
            self.send_at_fast("ATE0")
            
            # Enable BLE (skip if already enabled)
            ble_status = self.send_at_fast("AT+UBTLE?")
            if "0" in ble_status:
                self.send_at_fast("AT+UBTLE=2")
            
            # Set device name
            self.send_at_fast('AT+UBTLN="' + self.device_name + '"')
            
            # Set connection mode
            self.send_at_fast("AT+UBTCM=2")
            
            # Set advertising data (working format)
            self.send_at_fast("AT+UBTAD=02010604094454530416341230")
            
            # Start advertising
            self.send_at_fast("AT+UBTDM=3")
            
            print("✅ BLE configured successfully!")
            print("📱 Device: '" + self.device_name + "' - Ready for discovery")
            
            return True
            
        except Exception as e:
            print("❌ Configuration failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()
    
    def factory_reset_ultra_fast(self):
        """Ultra-fast factory reset"""
        print("🚀 Ultra-Fast Factory Reset")
        
        try:
            # Initialize UART
            self.uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
            
            # Quick AT test
            if "OK" not in self.send_at_fast("AT"):
                print("❌ Module not responding")
                return False
            
            # Disable echo
            self.send_at_fast("ATE0")
            
            # Factory reset
            if "OK" not in self.send_at_fast("AT+UFACTORY"):
                print("❌ Factory reset failed")
                return False
            
            # Wait minimal time
            time.sleep(1)
            
            # Fast BLE setup
            self.send_at_fast("AT+UBTLE=2")
            self.send_at_fast('AT+UBTLN="' + self.device_name + '"')
            self.send_at_fast("AT+UBTCM=2")
            self.send_at_fast("AT+UBTAD=02010604094454530416341230")
            self.send_at_fast("AT+UBTDM=3")
            self.send_at_fast("AT&W")
            
            print("✅ Factory reset completed!")
            print("📱 Device: '" + self.device_name + "' - Ready for discovery")
            
            return True
            
        except Exception as e:
            print("❌ Factory reset failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()
    
    def quick_status(self):
        """Quick status check"""
        print("🔍 Quick Status Check")
        
        try:
            self.uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
            
            if "OK" not in self.send_at_fast("AT"):
                print("❌ Module not responding")
                return False
            
            self.send_at_fast("ATE0")
            
            # Check BLE
            ble_status = self.send_at_fast("AT+UBTLE?")
            print("BLE: " + ble_status)
            
            # Check name
            name_status = self.send_at_fast("AT+UBTLN?")
            print("Name: " + name_status)
            
            # Start advertising
            self.send_at_fast("AT+UBTDM=3")
            print("✅ Advertising started")
            
            return True
            
        except Exception as e:
            print("❌ Status check failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()

def main():
    """Main function"""
    ble = EC200ABLEProduction(device_name="ANNA")
    
    print("🚀 EC200A BLE Production Tool")
    print("1. Ultra-fast configuration")
    print("2. Ultra-fast factory reset")
    print("3. Quick status")
    
    # Run ultra-fast configuration
    print("\nRunning ultra-fast configuration...")
    success = ble.configure_ble_ultra_fast()
    
    if success:
        print("\n🎉 Success! BLE module is ready!")
    else:
        print("\n💥 Failed. Check connections.")

if __name__ == "__main__":
    main() 