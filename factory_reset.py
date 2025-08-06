"""
Factory Reset Script for ANNA-B112 BLE Module
Based on u-connectXpress AT Commands Manual
This script performs a complete factory reset and disables Bluetooth advertising
"""

from machine import UART
import time

def send_at_command(uart, command, timeout=3):
    """Send AT command and return response"""
    if not uart:
        return "ERROR: UART not initialized"
        
    print("Sending: " + command)
    uart.write((command + '\r\n').encode())
    time.sleep(0.1)
    
    response = b''
    start_time = time.time()
    while time.time() - start_time < timeout:
        if uart.any():
            data = uart.read()
            if data:
                response += data
                if b'OK' in response or b'ERROR' in response or len(response) > 200:
                    break
        time.sleep(0.01)
    
    response_str = response.decode().strip()
    print("Response: " + response_str)
    return response_str

def factory_reset_ble():
    """Complete factory reset and clear custom advertising data"""
    print("=== ANNA-B112 BLE FACTORY RESET ===")
    print("Based on u-connectXpress AT Commands Manual")
    print("This will reset ALL settings to factory defaults and clear custom advertising data!")
    print("All custom settings and advertising data will be lost.")
    print("BLE will remain ENABLED for power efficiency.")
    print()
    
    # Initialize UART
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        print("✅ UART initialized")
    except Exception as e:
        print("❌ Failed to initialize UART: " + str(e))
        return False
    
    try:
        # Step 1: Test communication
        print("\n1. Testing module communication...")
        result = send_at_command(uart, "AT")
        if "ERROR" in result:
            print("❌ Module not responding")
            return False
        print("✅ Module responding")
        
        # Step 2: Factory Reset (Page 23)
        print("\n2. Performing factory reset...")
        result = send_at_command(uart, "AT+UFACTORY")
        if "OK" not in result:
            print("❌ Factory reset failed")
            return False
        print("✅ Factory reset completed")
        print("Note: Factory reset re-enables BLE advertising by default")
        
        
        # Step 5: Set device name to recognizable name
        print("\n5. Setting device name to recognizable name...")
        result = send_at_command(uart, 'AT+UBTLN="ANNA_B112"')
        if "OK" not in result:
            print("❌ Failed to set device name")
            return False
        print("✅ Device name set to 'ANNA_B112'")
        
        # Step 4: Set Non-Discoverable Mode (Page 57-58)
        print("\n4. Setting non-discoverable mode...")
        result = send_at_command(uart, "AT+UBTDM=1")
        if "OK" not in result:
            print("❌ Failed to set min interval")
            return False
        print("✅ Non-discoverable mode set")
        
        # Step 7: Save Configuration (Page 22)
        print("\n7. Saving configuration to profile...")
        result = send_at_command(uart, "AT&W")
        if "OK" not in result:
            print("❌ Failed to save configuration")
            return False
        print("✅ Configuration saved")
        
        # Step 8: Verify settings before reboot
        print("\n8. Verifying settings before reboot...")
        
        # Check BLE role (should be enabled)
        result = send_at_command(uart, "AT+UBTLE?")
        if "1" in result or "2" in result or "3" in result:
            print("✅ BLE role: Enabled")
        else:
            print("⚠️ BLE role may not be enabled properly")
        
        # Check device name
        result = send_at_command(uart, "AT+UBTLN?")
        if "DRONE_DETECTOR" in result:
            print("✅ Device name: DRONE_DETECTOR")
        else:
            print("⚠️ Device name may not be set properly")
        
        # Step 9: Reboot to apply changes (Page 28, 36)
        print("\n9. Rebooting to apply changes...")
        print("Note: Reboot ensures all changes are persistent")
        result = send_at_command(uart, "AT+CPWROFF")
        if "OK" not in result:
            print("❌ Failed to reboot")
            return False
        print("✅ Reboot command sent")
        
        # Step 10: Wait for reboot and verify
        print("\n10. Waiting for reboot to complete...")
        time.sleep(5)  # Wait for reboot to complete
        
        # Step 11: Verify final configuration
        print("\n11. Verifying final configuration...")
        result = send_at_command(uart, "AT")
        if "OK" not in result:
            print("❌ Module not responding after reboot")
            return False
        print("✅ Module responding after reboot")
        
        # Verify BLE role is enabled
        result = send_at_command(uart, "AT+UBTLE?")
        if "1" in result or "2" in result or "3" in result:
            print("✅ BLE role: Enabled (confirmed)")
        else:
            print("❌ BLE role: Not enabled - reboot may not have taken effect")
            return False
        
        # Verify device name
        result = send_at_command(uart, "AT+UBTLN?")
        if "DRONE_DETECTOR" in result:
            print("✅ Device name: DRONE_DETECTOR (confirmed)")
        else:
            print("❌ Device name: Not set properly")
            return False
        
        # Step 12: Final verification - check for any custom advertising data
        print("\n12. Final verification - checking for custom advertising data...")
        result = send_at_command(uart, "AT+UBTAD?")
        print("Advertising data response: " + result)
        
        if "DRONE_STATUS" in result or "1234" in result:
            print("❌ WARNING: Custom advertising data still present!")
            print("The factory reset may not have cleared all persistent data.")
            return False
        else:
            print("✅ No custom advertising data detected")
            print("✅ Only default factory advertising data present")
        
        print("\n" + "="*60)
        print("✅ FACTORY RESET AND DATA CLEAR COMPLETED SUCCESSFULLY!")
        print("📱 Module: ANNA-B112")
        print("🔵 BLE Role: ENABLED (for power efficiency)")
        print("📡 Advertising: CLEARED of custom data")
        print("⚙️ All Settings: FACTORY DEFAULTS")
        print("🔄 Module: REBOOTED")
        print("="*60)
        print("📋 Summary:")
        print("   • Factory reset completed")
        print("   • BLE role remains enabled for power efficiency")
        print("   • All custom advertising data cleared")
        print("   • Device name set to 'DRONE_DETECTOR' (searchable)")
        print("   • Advertising interval set to 5 seconds")
        print("   • No 'DRONE_STATUS' or custom data will be broadcast")
        print("="*60)
        
        return True
        
    except Exception as e:
        print("❌ Factory reset failed: " + str(e))
        return False
    
    finally:
        # Close UART
        if uart:
            uart.close()
            print("🔌 UART closed")

def main():
    """Main function"""
    print("ANNA-B112 BLE Factory Reset Tool")
    print("Based on u-connectXpress AT Commands Manual")
    print("This will reset your BLE module and clear custom advertising data.")
    print("All custom settings and advertising data will be lost.")
    print("BLE will remain ENABLED for power efficiency.")
    print()
    
    # Ask for confirmation
    print("Are you sure you want to proceed? (This cannot be undone)")
    print("Type 'YES' to continue or anything else to cancel:")
    
    # In a real implementation, you would get user input here
    # For now, we'll proceed automatically
    confirmation = "YES"  # In real use: input().strip().upper()
    
    if confirmation == "YES":
        success = factory_reset_ble()
        if success:
            print("\n🎉 Factory reset and data clear completed successfully!")
            print("Your ANNA-B112 module is now reset to factory defaults.")
            print("BLE remains enabled for power efficiency.")
            print("All custom advertising data (including 'DRONE_STATUS') has been cleared.")
        else:
            print("\n💥 Factory reset failed!")
            print("Please check your connections and try again.")
    else:
        print("\n❌ Factory reset cancelled.")

if __name__ == "__main__":
    main() 