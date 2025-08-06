"""
Test script to verify BLE is discoverable
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

def test_ble_discovery():
    """Test if BLE is discoverable"""
    
    print("=== Testing BLE Discovery ===")
    
    # Initialize UART
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        print("✅ UART initialized")
    except Exception as e:
        print("❌ Failed to initialize UART: " + str(e))
        return False
    
    try:
        # Test communication
        result = send_at_command(uart, "AT")
        if "ERROR" in result:
            print("❌ Module not responding")
            return False
        
        send_at_command(uart, "ATE0")  # Disable echo
        
        # Check current BLE status
        print("\n1. Checking BLE status...")
        result = send_at_command(uart, "AT+UBTLE?")
        if "2" in result:
            print("✅ BLE Peripheral role confirmed")
        else:
            print("❌ BLE role not set correctly")
            return False
        
        # Check device name
        print("\n2. Checking device name...")
        result = send_at_command(uart, "AT+UBTLN?")
        if "ANNA" in result:
            print("✅ Device name: 'ANNA'")
        else:
            print("⚠️ Device name may not be set correctly")
        
        # Check advertising data
        print("\n3. Checking advertising data...")
        result = send_at_command(uart, "AT+UBTAD?")
        if "02010604094454530416341230" in result:
            print("✅ Advertising data is set correctly")
        else:
            print("⚠️ Advertising data may not be set correctly")
        
        # Check if advertising is active
        print("\n4. Checking advertising status...")
        # Try to start advertising if not already active
        result = send_at_command(uart, "AT+UBTDM=3")
        if "OK" in result:
            print("✅ Advertising started successfully!")
        else:
            print("❌ Failed to start advertising")
            return False
        
        # Wait a moment for advertising to start
        print("\n5. Waiting for advertising to stabilize...")
        time.sleep(2)
        
        # Check advertising mode
        print("\n6. Verifying advertising mode...")
        # Note: There's no direct way to query advertising mode, but we can check if it's working
        
        print("\n=== BLE Discovery Test Complete ===")
        print("📱 Device should now be discoverable as 'ANNA'")
        print("📡 Advertising data: 02010604094454530416341230")
        print("🔍 Check your mobile BLE scanner app")
        print("   - Look for device named 'ANNA' or 'DTS'")
        print("   - Should appear every few seconds")
        
        return True
        
    except Exception as e:
        print("❌ Test failed: " + str(e))
        return False
    
    finally:
        if uart:
            uart.close()
            print("🔌 UART closed")

if __name__ == "__main__":
    test_ble_discovery() 