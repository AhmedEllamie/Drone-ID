"""
Optimized EC200A BLE Factory Reset and Configuration
Streamlined for maximum performance and reliability
"""

from machine import UART
import time

class EC200ABLEOptimizer:
    """Optimized BLE configuration for EC200A/ANNA-B4 modules"""
    
    def __init__(self, uart_port=UART.UART1, baud_rate=115200, device_name="ANNA"):
        self.uart_port = uart_port
        self.baud_rate = baud_rate
        self.device_name = device_name
        self.uart = None
        
    def send_at_command(self, command, timeout=2, silent=False):
        """Optimized AT command with reduced timeout and optional silent mode"""
        if not self.uart:
            return "ERROR: UART not initialized"
            
        if not silent:
            print("Sending: " + command)
            
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.02)  # Reduced delay for speed
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response or len(response) > 100:
                        break
            time.sleep(0.005)  # Faster polling
        
        response_str = response.decode().strip()
        
        if not silent:
            print("Response: " + response_str)
            print("---")
        
        return response_str
    
    def initialize_uart(self):
        """Initialize UART connection"""
        try:
            self.uart = UART(self.uart_port, self.baud_rate, 8, 0, 1, 0)
            print("✅ UART initialized (" + str(self.baud_rate) + " baud)")
            return True
        except Exception as e:
            print("❌ Failed to initialize UART: " + str(e))
            return False
    
    def test_communication(self):
        """Quick communication test"""
        result = self.send_at_command("AT", silent=True)
        return "OK" in result
    
    def configure_ble_fast(self, enable_advertising=True):
        """Fast BLE configuration without factory reset"""
        print("=== Fast BLE Configuration ===")
        
        if not self.initialize_uart():
            return False
        
        try:
            # Step 1: Quick communication test
            if not self.test_communication():
                print("❌ Module not responding")
                return False
            print("✅ Module responding")
            
            # Step 2: Disable echo for cleaner output
            self.send_at_command("ATE0", silent=True)
            
            # Step 3: Enable BLE (skip if already enabled)
            result = self.send_at_command("AT+UBTLE?", silent=True)
            if "0" in result:
                print("Enabling BLE...")
                self.send_at_command("AT+UBTLE=2", silent=True)
            else:
                print("✅ BLE already enabled")
            
            # Step 4: Set device name
            print("Setting device name...")
            self.send_at_command('AT+UBTLN="' + self.device_name + '"', silent=True)
            
            # Step 5: Set connection mode
            print("Setting connection mode...")
            self.send_at_command("AT+UBTCM=2", silent=True)
            
            # Step 6: Set advertising data (use working format)
            print("Setting advertising data...")
            working_adv_data = "02010604094454530416341230"
            self.send_at_command("AT+UBTAD=" + working_adv_data, silent=True)
            
            # Step 7: Start advertising if requested
            if enable_advertising:
                print("Starting advertising...")
                self.send_at_command("AT+UBTDM=3", silent=True)
            
            print("✅ Fast BLE configuration completed!")
            return True
            
        except Exception as e:
            print("❌ Configuration failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()
                print("🔌 UART closed")
    
    def factory_reset_optimized(self):
        """Optimized factory reset with minimal steps"""
        print("=== Optimized EC200A BLE Factory Reset ===")
        
        if not self.initialize_uart():
            return False
        
        try:
            # Step 1: Quick communication test
            if not self.test_communication():
                print("❌ Module not responding")
                return False
            print("✅ Module responding")
            
            # Step 2: Disable echo
            self.send_at_command("ATE0", silent=True)
            
            # Step 3: Factory reset
            print("Performing factory reset...")
            result = self.send_at_command("AT+UFACTORY")
            if "OK" not in result:
                print("❌ Factory reset failed")
                return False
            print("✅ Factory reset completed")
            
            # Step 4: Wait for stabilization
            print("Waiting for stabilization...")
            time.sleep(1.5)  # Reduced wait time
            
            # Step 5: Fast BLE configuration
            print("Configuring BLE...")
            self.send_at_command("AT+UBTLE=2", silent=True)  # Enable BLE
            self.send_at_command('AT+UBTLN="' + self.device_name + '"', silent=True)  # Set name
            self.send_at_command("AT+UBTCM=2", silent=True)  # Set connection mode
            
            # Step 6: Set advertising data
            working_adv_data = "02010604094454530416341230"
            self.send_at_command("AT+UBTAD=" + working_adv_data, silent=True)
            
            # Step 7: Start advertising
            print("Starting advertising...")
            self.send_at_command("AT+UBTDM=3", silent=True)
            
            # Step 8: Save configuration
            print("Saving configuration...")
            self.send_at_command("AT&W", silent=True)
            
            print("✅ Optimized factory reset completed!")
            print("📱 Device should be discoverable as '" + self.device_name + "'")
            
            return True
            
        except Exception as e:
            print("❌ Factory reset failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()
                print("🔌 UART closed")
    
    def verify_configuration(self):
        """Quick verification of BLE configuration"""
        print("=== Verifying BLE Configuration ===")
        
        if not self.initialize_uart():
            return False
        
        try:
            if not self.test_communication():
                print("❌ Module not responding")
                return False
            
            self.send_at_command("ATE0", silent=True)
            
            # Check BLE status
            result = self.send_at_command("AT+UBTLE?", silent=True)
            if "2" in result:
                print("✅ BLE: Enabled (Peripheral)")
            else:
                print("❌ BLE: Not properly configured")
                return False
            
            # Check device name
            result = self.send_at_command("AT+UBTLN?", silent=True)
            if self.device_name in result:
                print("✅ Device name: '" + self.device_name + "'")
            else:
                print("⚠️ Device name: " + result)
            
            # Check advertising data
            result = self.send_at_command("AT+UBTAD?", silent=True)
            if "02010604094454530416341230" in result:
                print("✅ Advertising data: Set correctly")
            else:
                print("⚠️ Advertising data: " + result)
            
            # Start advertising if not already active
            print("Starting advertising...")
            self.send_at_command("AT+UBTDM=3", silent=True)
            
            print("✅ Configuration verified!")
            print("📱 Device should be discoverable as '" + self.device_name + "'")
            
            return True
            
        except Exception as e:
            print("❌ Verification failed: " + str(e))
            return False
        finally:
            if self.uart:
                self.uart.close()
                print("🔌 UART closed")

def main():
    """Main function with menu options"""
    print("🚀 EC200A BLE Optimizer")
    print("Choose operation:")
    print("1. Fast BLE configuration (no factory reset)")
    print("2. Optimized factory reset")
    print("3. Verify configuration")
    print("4. Quick status check")
    
    # For now, run optimized factory reset
    optimizer = EC200ABLEOptimizer(device_name="ANNA")
    
    print("\nRunning optimized factory reset...")
    success = optimizer.factory_reset_optimized()
    
    if success:
        print("\n🎉 Success! Your BLE module is now optimized and ready!")
    else:
        print("\n💥 Configuration failed. Check connections and try again.")

if __name__ == "__main__":
    main() 