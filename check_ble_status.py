"""
Quick BLE Status Check
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
    return response_str

def check_ble_status():
    """Quick BLE status check"""
    
    print("=== QUICK BLE STATUS CHECK ===")
    
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        
        # Check BLE status
        print("\n1. BLE Status:")
        result = send_at_command(uart, "AT+UBTLE?")
        if "2" in result:
            print("✅ BLE is ENABLED (Peripheral mode)")
        elif "1" in result:
            print("⚠️ BLE is ENABLED (Central mode)")
        elif "0" in result:
            print("❌ BLE is DISABLED - This is why it's not discoverable!")
            print("   Enabling BLE...")
            result = send_at_command(uart, "AT+UBTLE=2")
            if "OK" in result:
                print("✅ BLE enabled successfully")
            else:
                print("❌ Failed to enable BLE")
        else:
            print("❌ Could not determine BLE status")
        
        # Check device name
        print("\n2. Device Name:")
        result = send_at_command(uart, "AT+UBTLN?")
        if "ANNA" in result:
            print("✅ Device name: 'ANNA'")
        else:
            print("⚠️ Device name not set correctly")
        
        # Check advertising data
        print("\n3. Advertising Data:")
        result = send_at_command(uart, "AT+UBTAD?")
        if "02010604094454530416341230" in result:
            print("✅ Advertising data is set")
        else:
            print("⚠️ Advertising data not set correctly")
        
        # Check if advertising is active
        print("\n4. Starting Advertising:")
        result = send_at_command(uart, "AT+UBTDM=3")
        if "OK" in result:
            print("✅ Advertising started")
            print("📱 Device should now be discoverable as 'ANNA'")
        else:
            print("❌ Failed to start advertising")
        
        uart.close()
        
    except Exception as e:
        print("❌ Check failed: " + str(e))

if __name__ == "__main__":
    check_ble_status() 