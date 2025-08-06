"""
Simple Counter Beacon
Copies only the start_advertising method from anna_advertising_beacon.py
No initialization - just send the counter!
"""

from machine import UART
import time

def string_to_hex(text):
    """Convert string to hex (QuecPython compatible)"""
    hex_result = ""
    for char in text:
        hex_val = hex(ord(char))[2:]  # Remove '0x' prefix
        if len(hex_val) == 1:
            hex_val = '0' + hex_val
        hex_result += hex_val
    return hex_result

def create_advertising_data(message):
    """Create GAP advertising data with simplified format"""
    
    # Conservative message length
    max_message_length = 15
    if len(message) > max_message_length:
        message = message[:max_message_length]
    
    msg_hex = string_to_hex(message)
    
    # Block 1: AD Flags (mandatory) - 3 bytes total
    flags_block = "020106"  # Length=2, Type=0x01, Flags=0x06
    
    # Block 2: Complete Local Name (message in device name)
    name_length = len(message) + 1  # +1 for type byte
    name_length_hex = "{:02X}".format(name_length)
    name_block = name_length_hex + "09" + msg_hex
    
    advertising_data = flags_block + name_block
    
    return advertising_data

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

def start_advertising(uart, message):
    """Start advertising with specified message (copied from class)"""
    try:
        print("=== Starting Advertising ===")
        
        # Set advertising data
        adv_data = create_advertising_data(message)
        print("Advertising data: " + adv_data)
        print("Data length: " + str(len(adv_data) // 2) + " bytes")
        
        # Set advertising data
        result = send_at_command(uart, "AT+UBTAD=" + adv_data)
        
        if "ERROR" in result:
            print("Failed to set advertising data")
            return False
        
        # Start connectable advertising
        result = send_at_command(uart, "AT+UBTDM=3")
        
        if "OK" in result:
            print("Beacon started! Broadcasting: " + message)
            print("📱 Device appears as: '" + message + "' (changing)")
            return True
        else:
            print("Failed to start advertising")
            return False
            
    except Exception as e:
        print("Start advertising failed: " + str(e))
        return False

def update_message(uart, message):
    """Update the advertising message"""
    try:
        # Create new advertising data
        adv_data = create_advertising_data(message)
        result = send_at_command(uart, "AT+UBTAD=" + adv_data)
        
        if "OK" in result:
            return True
        else:
            return False
            
    except Exception as e:
        print("Update message failed: " + str(e))
        return False

def main():
    """Simple counter advertising without class initialization"""
    print("🚀 Simple Counter Beacon")
    print("Using only start_advertising method - no hardware reset!")
    print("Just send the counter!")
    
    # Initialize UART (minimal setup)
    try:
        uart = UART(UART.UART1, 115200, 8, 0, 1, 0)
        print("✅ UART initialized")
    except Exception as e:
        print("❌ Failed to initialize UART: " + str(e))
        return
    
    # Test communication
    result = send_at_command(uart, "AT")
    if "ERROR" in result:
        print("❌ Module not responding")
        return
    
    # Start advertising with initial counter
    print("\nStarting counter advertising...")
    counter = 0
    initial_message = str(counter)
    
    if not start_advertising(uart, initial_message):
        print("❌ Failed to start advertising")
        return
    
    print("✅ Counter advertising started!")
    print("🔢 Counter updates every 5 seconds")
    print("⏹️ Press Ctrl+C to stop")
    
    try:
        # Main loop - update counter every 5 seconds
        while True:
            time.sleep(5)  # Wait 5 seconds
            
            # Increment counter
            counter += 1
            counter_message = str(counter)
            
            # Update advertising message
            if update_message(uart, counter_message):
                print("🔢 Counter: " + counter_message)
            else:
                print("⚠️ Failed to update counter")
                
    except KeyboardInterrupt:
        print("\n⏹️ Stopping counter beacon...")
        send_at_command(uart, "AT+UBTDM=0")  # Stop advertising
        uart.close()
        print("✅ Counter beacon stopped")

if __name__ == "__main__":
    main() 