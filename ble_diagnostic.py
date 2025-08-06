"""
Comprehensive BLE Diagnostic Script
Troubleshoots why ublox BLE module isn't discoverable
"""

from machine import UART
import time

def send_at_command(uart, command, timeout=3):
    """Send AT command and return response"""
    if not uart:
        return "ERROR: UART not initialized"
        
    print("Sending: " + command)
    uart.write((command + '\r\n').encode())
    time.sleep(0.05)
    
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
    print("---")
    
    return response_str

def ble_diagnostic():
    """Comprehensive BLE diagnostic"""
    
    print("=== BLE DIAGNOSTIC - Troubleshooting Discovery Issues ===")
    
    # Initialize UART
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        print("✅ UART initialized")
    except Exception as e:
        print("❌ Failed to initialize UART: " + str(e))
        return False
    
    try:
        # Test 1: Basic communication
        print("\n1. Testing basic communication...")
        result = send_at_command(uart, "AT")
        if "ERROR" in result:
            print("❌ Module not responding to AT")
            return False
        print("✅ Module responds to AT commands")
        
        send_at_command(uart, "ATE0")  # Disable echo
        
        # Test 2: Check if BLE is enabled
        print("\n2. Checking BLE status...")
        result = send_at_command(uart, "AT+UBTLE?")
        if "2" in result:
            print("✅ BLE is enabled (Peripheral mode)")
        elif "1" in result:
            print("⚠️ BLE is enabled (Central mode)")
        elif "0" in result:
            print("❌ BLE is DISABLED - this is the problem!")
            print("   Enabling BLE...")
            result = send_at_command(uart, "AT+UBTLE=2")
            if "OK" in result:
                print("✅ BLE enabled successfully")
            else:
                print("❌ Failed to enable BLE")
                return False
        else:
            print("❌ Could not determine BLE status")
            return False
        
        # Test 3: Check device name
        print("\n3. Checking device name...")
        result = send_at_command(uart, "AT+UBTLN?")
        if "ANNA" in result:
            print("✅ Device name: 'ANNA'")
        else:
            print("⚠️ Device name not set correctly")
            print("   Setting device name...")
            result = send_at_command(uart, 'AT+UBTLN="ANNA"')
            if "OK" in result:
                print("✅ Device name set to 'ANNA'")
            else:
                print("❌ Failed to set device name")
        
        # Test 4: Check connection mode
        print("\n4. Checking connection mode...")
        # Note: No direct query for connection mode, so we'll set it
        result = send_at_command(uart, "AT+UBTCM=2")
        if "OK" in result:
            print("✅ Connection mode set to connectable")
        else:
            print("❌ Failed to set connection mode")
            return False
        
        # Test 5: Check advertising data
        print("\n5. Checking advertising data...")
        result = send_at_command(uart, "AT+UBTAD?")
        if "02010604094454530416341230" in result:
            print("✅ Advertising data is set correctly")
        else:
            print("⚠️ Advertising data not set correctly")
            print("   Setting advertising data...")
            working_adv_data = "02010604094454530416341230"
            result = send_at_command(uart, "AT+UBTAD=" + working_adv_data)
            if "OK" in result:
                print("✅ Advertising data set successfully")
            else:
                print("❌ Failed to set advertising data")
        
        # Test 6: Check advertising mode
        print("\n6. Checking advertising mode...")
        # Try different advertising modes
        print("   Testing advertising mode 3 (General discoverable)...")
        result = send_at_command(uart, "AT+UBTDM=3")
        if "OK" in result:
            print("✅ Advertising mode 3 set successfully")
        else:
            print("❌ Failed to set advertising mode 3")
            return False
        
        # Test 7: Check if advertising is actually working
        print("\n7. Verifying advertising is active...")
        print("   Waiting 5 seconds for advertising to start...")
        time.sleep(5)
        
        # Test 8: Try alternative advertising modes
        print("\n8. Testing alternative advertising modes...")
        
        # Mode 1: Limited discoverable
        print("   Testing mode 1 (Limited discoverable)...")
        result = send_at_command(uart, "AT+UBTDM=1")
        if "OK" in result:
            print("✅ Mode 1 set - try scanning now")
            time.sleep(3)
        else:
            print("❌ Mode 1 failed")
        
        # Mode 2: Non-discoverable
        print("   Testing mode 2 (Non-discoverable)...")
        result = send_at_command(uart, "AT+UBTDM=2")
        if "OK" in result:
            print("✅ Mode 2 set - device should NOT be visible")
            time.sleep(3)
        else:
            print("❌ Mode 2 failed")
        
        # Back to mode 3
        print("   Back to mode 3 (General discoverable)...")
        result = send_at_command(uart, "AT+UBTDM=3")
        if "OK" in result:
            print("✅ Mode 3 set - device should be visible")
        else:
            print("❌ Mode 3 failed")
        
        # Test 9: Check for any error states
        print("\n9. Checking for error states...")
        result = send_at_command(uart, "AT+UBTLE?")
        print("   Current BLE status: " + result)
        
        # Test 10: Final verification
        print("\n10. Final verification...")
        print("   Current configuration:")
        send_at_command(uart, "AT+UBTLE?")
        send_at_command(uart, "AT+UBTLN?")
        send_at_command(uart, "AT+UBTAD?")
        
        print("\n=== DIAGNOSTIC COMPLETE ===")
        print("📱 If device still not visible, try:")
        print("   1. Power cycle the module")
        print("   2. Check if BLE is enabled in your phone")
        print("   3. Try different BLE scanner apps")
        print("   4. Check if module has proper power supply")
        print("   5. Verify antenna connection (if external)")
        
        return True
        
    except Exception as e:
        print("❌ Diagnostic failed: " + str(e))
        return False
    
    finally:
        if uart:
            uart.close()
            print("🔌 UART closed")

def force_ble_reset():
    """Force a complete BLE reset"""
    
    print("=== FORCE BLE RESET ===")
    
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        
        # Step 1: Disable BLE completely
        print("1. Disabling BLE...")
        send_at_command(uart, "AT+UBTLE=0")
        time.sleep(2)
        
        # Step 2: Re-enable BLE
        print("2. Re-enabling BLE...")
        send_at_command(uart, "AT+UBTLE=2")
        time.sleep(2)
        
        # Step 3: Set basic configuration
        print("3. Setting basic configuration...")
        send_at_command(uart, 'AT+UBTLN="ANNA"')
        send_at_command(uart, "AT+UBTCM=2")
        send_at_command(uart, "AT+UBTAD=02010604094454530416341230")
        
        # Step 4: Start advertising
        print("4. Starting advertising...")
        send_at_command(uart, "AT+UBTDM=3")
        
        print("✅ BLE reset complete - try scanning now")
        
        uart.close()
        return True
        
    except Exception as e:
        print("❌ BLE reset failed: " + str(e))
        return False

if __name__ == "__main__":
    print("Choose diagnostic option:")
    print("1. Run comprehensive diagnostic")
    print("2. Force BLE reset")
    
    # For now, run diagnostic
    ble_diagnostic() 