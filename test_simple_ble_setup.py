"""
Simple test to verify the correct BLE setup sequence
Based on working patterns from anna_advertising_beacon.py
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

def test_ble_setup_sequence():
    """Test the correct BLE setup sequence"""
    
    print("=== Testing BLE Setup Sequence ===")
    
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
        
        # Step 1: Set BLE role
        print("\n1. Setting BLE Peripheral Role...")
        result = send_at_command(uart, "AT+UBTLE=2")
        if "OK" not in result:
            print("❌ Failed to set BLE role")
            return False
        print("✅ BLE Peripheral role set")
        
        # Step 2: Set device name
        print("\n2. Setting device name...")
        result = send_at_command(uart, 'AT+UBTLN="ANNA"')
        if "OK" not in result:
            print("❌ Failed to set device name")
            return False
        print("✅ Device name set to 'ANNA'")
        
        # Step 3: Set connection mode (IMPORTANT: Must be done before advertising data)
        print("\n3. Setting connection mode...")
        result = send_at_command(uart, "AT+UBTCM=2")
        if "OK" not in result:
            print("❌ Failed to set connection mode")
            return False
        print("✅ Connection mode set to connectable")
        
        # Step 4: Test advertising data (should work now)
        print("\n4. Testing advertising data...")
        working_adv_data = "02010604094454530416341230"
        result = send_at_command(uart, "AT+UBTAD=" + working_adv_data)
        if "OK" in result:
            print("✅ Working advertising data set successfully!")
            print("   Data: " + working_adv_data)
        else:
            print("❌ Still failed to set advertising data")
            return False
        
        # Step 5: Verify the data was set
        print("\n5. Verifying advertising data...")
        result = send_at_command(uart, "AT+UBTAD?")
        if "OK" in result:
            print("✅ Current advertising data: " + result)
        else:
            print("❌ Could not read advertising data")
        
        # Step 6: Start advertising
        print("\n6. Starting advertising...")
        result = send_at_command(uart, "AT+UBTDM=3")
        if "OK" in result:
            print("✅ Advertising started successfully!")
        else:
            print("❌ Failed to start advertising")
            return False
        
        print("\n=== Test Complete - BLE Setup Sequence Works! ===")
        return True
        
    except Exception as e:
        print("❌ Test failed: " + str(e))
        return False
    
    finally:
        if uart:
            uart.close()
            print("🔌 UART closed")

if __name__ == "__main__":
    test_ble_setup_sequence() 