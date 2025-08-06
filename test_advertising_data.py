"""
Simple test script to verify advertising data formats for EC200A/ANNA-B4
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

def string_to_hex(text):
    """Convert string to hex (QuecPython compatible)"""
    hex_result = ""
    for char in text:
        hex_val = hex(ord(char))[2:]  # Remove '0x' prefix
        if len(hex_val) == 1:
            hex_val = '0' + hex_val
        hex_result += hex_val
    return hex_result

def test_advertising_data_formats():
    """Test different advertising data formats"""
    
    print("=== Testing Advertising Data Formats ===")
    
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
        
        # Test 1: Minimal flags only
        print("\n1. Testing minimal flags only...")
        result = send_at_command(uart, "AT+UBTAD=020106")
        if "OK" in result:
            print("✅ Minimal flags format works")
        else:
            print("❌ Minimal flags format failed")
        
        # Test 2: Device name format (like working example)
        print("\n2. Testing device name format...")
        device_name_hex = string_to_hex("ANNA")
        device_name_data = "0201060409" + device_name_hex
        result = send_at_command(uart, "AT+UBTAD=" + device_name_data)
        if "OK" in result:
            print("✅ Device name format works")
        else:
            print("❌ Device name format failed")
        
        # Test 3: Service data format (like current working data)
        print("\n3. Testing service data format...")
        service_data = "02010604094454530416341230"  # Flags + "DTS" + Service Data
        result = send_at_command(uart, "AT+UBTAD=" + service_data)
        if "OK" in result:
            print("✅ Service data format works")
        else:
            print("❌ Service data format failed")
        
        # Test 4: Clear advertising data
        print("\n4. Testing clear advertising data...")
        result = send_at_command(uart, "AT+UBTAD=")
        if "OK" in result:
            print("✅ Clear advertising data works")
        else:
            print("❌ Clear advertising data failed")
        
        # Test 5: Verify current data
        print("\n5. Verifying current advertising data...")
        result = send_at_command(uart, "AT+UBTAD?")
        if "OK" in result:
            print("✅ Current advertising data: " + result)
        else:
            print("❌ Could not read advertising data")
        
        print("\n=== Test Complete ===")
        return True
        
    except Exception as e:
        print("❌ Test failed: " + str(e))
        return False
    
    finally:
        if uart:
            uart.close()
            print("🔌 UART closed")

if __name__ == "__main__":
    test_advertising_data_formats() 