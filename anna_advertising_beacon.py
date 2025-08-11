"""
ANNA-B4 BLE Advertising Beacon for integration with other code

Author: Ahmed Ellamie
Email: ahmed.ellamiee@gmail.com
"""

from machine import UART
import time

class BLEAdvertisingBeacon:
    """Standalone ANNA-B4 BLE Advertising Beacon for integration with other code"""
    
    def __init__(self, uart_port=UART.UART1, baud_rate=115200, device_name="IO_BLE", use_extended_advertising=False, message_in_device_name=False):
        """Initialize the BLE beacon with UART configuration
        
        Args:
            uart_port: UART port to use
            baud_rate: UART baud rate  
            device_name: Static device name (used when message_in_device_name=False)
            use_extended_advertising: Enable 252-byte extended advertising (vs 28-byte standard)
            message_in_device_name: Put changing message in device name (0x09) instead of manufacturer data (0xFF)
        """
        self.uart_port = uart_port
        self.baud_rate = baud_rate
        self.uart = None
        self.is_connected = False
        self.connection_handle = None
        self.device_name = device_name
        self.current_message = ""
        self.is_advertising = False
        self.is_initialized = False
        self.need_resume_advertising = False
        self.use_extended_advertising = use_extended_advertising
        self.message_in_device_name = message_in_device_name
        self.max_data_length = 252 if use_extended_advertising else 28
        
    def send_at_command(self, command, timeout=3):
        """Send AT command and return response"""
        if not self.uart:
            return "ERROR: UART not initialized"
            
        print("Sending: " + command)
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.05)  # Reduced from 0.2 to 0.05 for faster operation
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response or len(response) > 200:
                        break
            time.sleep(0.01)  # Reduced from 0.1 to 0.01 for faster polling
        
        response_str = response.decode().strip()
        print("Response: " + response_str)
        print("---")
        
        # Check for events in the response
        self._process_events_in_response(response_str)
        
        return response_str

    # Ultra-fast version of send_at_command for high-frequency updates
    def send_at_command_fast(self, command, timeout=0.3):
        """Ultra-fast AT command for high-frequency updates with minimal latency"""
        if not self.uart:
            return "ERROR: UART not initialized"
            
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.01)  # Minimal delay for maximum speed
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response:
                        break
            time.sleep(0.002)  # Ultra-fast polling (2ms)
        
        response_str = response.decode().strip()
        
        # Check for events in the response
        self._process_events_in_response(response_str)
        
        return response_str
    
    def _process_events_in_response(self, response):
        """Process BLE events that might appear in AT command responses"""
        lines = response.split('\n')
        for line in lines:
            line = line.strip()
            if "+UUBTACLC" in line:
                # Connection event: +UUBTACLC:<peer_handle>,<own_handle>,<peer_address>
                parts = line.split(':')
                if len(parts) >= 2:
                    conn_parts = parts[1].split(',')
                    if len(conn_parts) >= 2:
                        self.connection_handle = conn_parts[0]  # peer_handle
                        self.is_connected = True
                        print("Device connected! Handle: " + str(self.connection_handle))
            elif "+UUBTACLD" in line:
                # Disconnection event
                print("Device disconnected")
                self.is_connected = False
                self.connection_handle = None
                # Set flag to resume advertising (will be handled in check_events)
                if self.is_advertising:
                    self.need_resume_advertising = True
    
    def string_to_hex(self, text):
        """Convert string to hex (QuecPython compatible)"""
        hex_result = ""
        for char in text:
            hex_val = hex(ord(char))[2:]  # Remove '0x' prefix
            if len(hex_val) == 1:
                hex_val = '0' + hex_val
            hex_result += hex_val
        return hex_result
    
    def create_advertising_data(self, message, debug=False):
        """Create GAP advertising data with simplified format"""
        
        # For debugging - show what we're creating
        original_msg_len = len(message)
        
        # Much more conservative message length to avoid errors
        max_message_length = 15  # Very conservative for 28-byte total limit
        if len(message) > max_message_length:
            message = message[:max_message_length]
            if debug:
                print("Warning: Message truncated from " + str(original_msg_len) + " to " + str(len(message)) + " bytes")
        
        msg_hex = self.string_to_hex(message)
        
        # Block 1: AD Flags (mandatory) - 3 bytes total
        flags_block = "020106"  # Length=2, Type=0x01, Flags=0x06
        
        advertising_data = flags_block
        
        if self.message_in_device_name:
            # Mode 1: Message in Device Name (0x09)
            name_length = len(message) + 1  # +1 for type byte
            name_length_hex = "{:02X}".format(name_length)
            name_block = name_length_hex + "09" + msg_hex
            advertising_data += name_block
            
        else:
            # Mode 2: Shortened device name to make room for manufacturer data
            # Use shorter name when we have a message to broadcast
            if len(message) > 4:  # If we have actual telemetry data
                short_name = "DTS"  # Much shorter to save space
            else:
                short_name = self.device_name  # Use full name for initial messages
                
            # Block 2: Complete Local Name
            device_name_hex = self.string_to_hex(short_name)
            name_length = len(short_name) + 1  # +1 for type byte  
            name_length_hex = "{:02X}".format(name_length)
            name_block = name_length_hex + "09" + device_name_hex
            advertising_data += name_block
            
            # Block 3: Manufacturer Specific Data (simplified)
            # Calculate if we can fit manufacturer data
            company_id = "FFFF"  # Test/Development Company ID
            mfg_data_content = company_id + msg_hex
            mfg_length = len(mfg_data_content) // 2 + 1  # +1 for type byte (0xFF)
            mfg_length_hex = "{:02X}".format(mfg_length)
            mfg_block = mfg_length_hex + "FF" + mfg_data_content
            
            # Check if adding this block would exceed limit
            current_size = len(advertising_data) // 2
            mfg_block_size = len(mfg_block) // 2
            total_size = current_size + mfg_block_size
            
            if debug:
                print("  Size calculation:")
                print("    Current data: {} bytes".format(current_size))
                print("    Manufacturer block: {} bytes".format(mfg_block_size))
                print("    Total would be: {} bytes".format(total_size))
                print("    Limit: {} bytes".format(self.max_data_length))
            
            if total_size <= self.max_data_length:
                advertising_data += mfg_block
                if debug:
                    print("  ✅ Added manufacturer data: {} bytes".format(mfg_block_size))
            else:
                if debug:
                    print("  ❌ Skipping manufacturer data - would exceed size limit")
                    print("    Need {} bytes but only have {}".format(total_size, self.max_data_length))
        
        # Check total length
        total_length = len(advertising_data) // 2
        
        # Debug info (only when requested)
        if debug:
            print("📊 Advertising data debug:")
            print("  Message: '" + message + "' (" + str(len(message)) + " bytes)")
            print("  Total packet: " + str(total_length) + " bytes")
            print("  Limit: " + str(self.max_data_length) + " bytes")
            print("  Hex data: " + advertising_data)
        
        # Final safety check - hard truncate if still too long
        if total_length > self.max_data_length:
            if debug:
                print("❌ Packet still too large! Hard truncating...")
            max_hex_length = self.max_data_length * 2
            advertising_data = advertising_data[:max_hex_length]
            if debug:
                print("  Truncated to: " + str(len(advertising_data) // 2) + " bytes")
        
        return advertising_data
    
    def initialize(self):
        """Initialize UART and configure BLE parameters"""
        if self.is_initialized:
            return True
            
        try:
            # Initialize UART
            self.uart = UART(self.uart_port, self.baud_rate, 8, 0, 1, 0)
            
            print("=== Initializing BLE Advertising Beacon ===")
            
            # Basic AT setup
            result = self.send_at_command("AT")
            if "ERROR" in result:
                return False
                
            self.send_at_command("ATE0")  # Disable echo
            
            # Set connectable mode (allows connections)
            self.send_at_command("AT+UBTCM=2")
            
            # Set device name (only used in static mode)
            self.send_at_command('AT+UBTLN="' + self.device_name + '"')
            
            # Configure advertising parameters
            print("--- Configuring Advertising Parameters ---")
            
            # Test Parameter 26: LL PDU payload size (Data Length Extension)
            # This might affect advertising data capacity
            if self.use_extended_advertising:
                print("Testing Data Length Extension (parameter 26)")
                result = self.send_at_command("AT+UBTLECFG=26,1")  # Enable DLE + request max sizes
                if "ERROR" in result:
                    print("Warning: Data Length Extension not supported")
                    result = self.send_at_command("AT+UBTLECFG=26,0")  # Try accept mode
                    if "ERROR" in result:
                        print("Warning: Parameter 26 not supported, using standard limits")
                        self.use_extended_advertising = False
                        self.max_data_length = 28
                else:
                    print("✅ Data Length Extension enabled")
            
            # Set advertising interval (optimized for 5-second updates)
            # Intervals are in 0.625ms units, so: 
            # For 5 seconds exactly: 5000ms ÷ 0.625ms = 8000 units
            self.send_at_command("AT+UBTLECFG=1,8000")  # Min interval: 5 seconds exactly
            self.send_at_command("AT+UBTLECFG=2,8000")  # Max interval: 5 seconds exactly (same as min)
            
            # Set advertising channels (all channels for maximum reliability)
            self.send_at_command("AT+UBTLECFG=3,7")
            
            # Optimize connection parameters for low latency
            self.send_at_command("AT+UBTLECFG=4,6")   # Connection interval min (7.5ms)
            self.send_at_command("AT+UBTLECFG=5,6")   # Connection interval max (7.5ms)
            self.send_at_command("AT+UBTLECFG=6,0")   # Slave latency (0 for lowest latency)
            self.send_at_command("AT+UBTLECFG=7,100") # Supervision timeout (1000ms)

            
            self.is_initialized = True
            
            # Print configuration summary
            mode_desc = "Device Name (0x09)" if self.message_in_device_name else "Manufacturer Data (0xFF)"
            max_bytes = self.max_data_length
            print("BLE beacon initialized successfully")
            print("📡 Mode: Messages in " + mode_desc)
            print("📏 Max data: " + str(max_bytes) + " bytes")
            print("📱 Static name: '" + self.device_name + "'")
            
            return True
            
        except Exception as e:
            print("Initialization failed: " + str(e))
            return False
    
    def start_advertising(self, initial_message="hello"):
        """Start advertising with specified message"""
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        try:
            print("=== Starting Advertising ===")
            
            # Set advertising data with debug enabled
            adv_data = self.create_advertising_data(initial_message, debug=True)
            print("Final advertising data: " + adv_data)
            print("Data length: " + str(len(adv_data) // 2) + " bytes")
            
            # Try to set advertising data, with fallback if it fails
            result = self.send_at_command("AT+UBTAD=" + adv_data)
            
            # If that fails, try a simpler approach
            if "ERROR" in result:
                print("Trying simpler advertising data format...")
                # Use just device name without manufacturer data
                simple_data = "0201060D09" + self.string_to_hex(self.device_name)
                result = self.send_at_command("AT+UBTAD=" + simple_data)
            
            if "ERROR" in result:
                print("Failed to set advertising data")
                return False
            
            # Start connectable advertising
            result = self.send_at_command("AT+UBTDM=3")
            
            if "OK" in result:
                self.is_advertising = True
                self.current_message = initial_message
                print("Beacon started! Broadcasting: " + initial_message)
                
                if self.message_in_device_name:
                    print("📱 Device appears as: '" + initial_message + "' (changing)")
                    print("📡 Data location: Complete Local Name (0x09)")
                else:
                    print("📱 Device appears as: '" + self.device_name + "' (static)")
                    print("📡 Data location: Manufacturer Data (0xFF)")
                
                return True
            else:
                print("Failed to start advertising")
                return False
                
        except Exception as e:
            print("Start advertising failed: " + str(e))
            return False
    
    def stop_advertising(self):
        """Stop advertising"""
        if self.is_advertising:
            result = self.send_at_command("AT+UBTDM=0")  # Stop advertising
            if "OK" in result:
                self.is_advertising = False
                print("📡 Advertising stopped")
                return True
            else:
                print("Failed to stop advertising")
                return False
        return True
    
    def update_message(self, message):
        """Update the advertising message (only when not connected)"""
        if not self.is_advertising:
            print("Beacon not advertising. Call start_advertising() first.")
            return False
            
        if self.is_connected:
            return True  # Return True but skip the update (device connected)
        
        try:
            # Use Service Data for better bandwidth (18 bytes vs 12 bytes)
            adv_data = self.create_advertising_data_service_data(message)
            result = self.send_at_command_fast("AT+UBTAD=" + adv_data)
            
            if "OK" in result:
                self.current_message = message
                return True
            else:
                return False
                
        except Exception as e:
            print("Update message failed: " + str(e))
            return False

    def create_advertising_data_fast(self, message):
        """Fast version of create_advertising_data for high-frequency updates"""
        
        # Conservative message length for speed and reliability
        max_message_length = 12  # Conservative for fast updates
        if len(message) > max_message_length:
            message = message[:max_message_length]
        
        msg_hex = self.string_to_hex(message)
        
        # Block 1: AD Flags (mandatory) - cached
        flags_block = "020106"  # Length=2, Type=0x01, Flags=0x06
        
        advertising_data = flags_block
        
        if self.message_in_device_name:
            # Mode 1: Message in Device Name (0x09)
            name_length = len(message) + 1
            name_length_hex = "{:02X}".format(name_length)
            name_block = name_length_hex + "09" + msg_hex
            advertising_data += name_block
            
        else:
            # Mode 2: Optimized for fast updates
            # Use very short static name to maximize data space
            short_name = "DTS"  # Fixed short name for speed
            
            # Block 2: Complete Local Name (FIXED)
            device_name_hex = "445453"  # Pre-calculated hex for "DTS"
            name_length = len(short_name) + 1  # +1 for type byte
            name_length_hex = "{:02X}".format(name_length)
            name_block = name_length_hex + "09" + device_name_hex  # FIXED: proper format
            advertising_data += name_block
            
            # Block 3: Manufacturer Specific Data (simplified)
            company_id = "FFFF"  # Test/Development Company ID
            mfg_data_content = company_id + msg_hex
            mfg_length = len(mfg_data_content) // 2 + 1
            mfg_length_hex = "{:02X}".format(mfg_length)
            mfg_block = mfg_length_hex + "FF" + mfg_data_content
            
            # Fast size check (simplified)
            current_size = len(advertising_data) // 2
            mfg_block_size = len(mfg_block) // 2
            
            if current_size + mfg_block_size <= self.max_data_length:
                advertising_data += mfg_block
        
        return advertising_data
    
    def create_advertising_data_service_data(self, message):
        """Use Service Data for more bandwidth (up to ~20 bytes vs 12 bytes)"""
        
        # Service Data can hold more data than manufacturer data
        max_message_length = 18  # Much more room with Service Data
        if len(message) > max_message_length:
            message = message[:max_message_length]
        
        msg_hex = self.string_to_hex(message)
        
        # Block 1: AD Flags (mandatory)
        flags_block = "020106"  # Length=2, Type=0x01, Flags=0x06
        
        advertising_data = flags_block
        
        # Block 2: Short device name to save space
        short_name = "DTS"
        device_name_hex = "445453"  # Pre-calculated hex for "DTS"
        name_length = len(short_name) + 1
        name_length_hex = "{:02X}".format(name_length)
        name_block = name_length_hex + "09" + device_name_hex
        advertising_data += name_block
        
        # Block 3: Service Data (0x16) - More space than manufacturer data
        # Custom UUID: 0x1234 (16-bit service UUID)
        service_uuid = "3412"  # Little endian format for 0x1234
        service_data_content = service_uuid + msg_hex
        service_length = len(service_data_content) // 2 + 1  # +1 for type byte
        service_length_hex = "{:02X}".format(service_length)
        service_block = service_length_hex + "16" + service_data_content
        
        # Check size
        current_size = len(advertising_data) // 2
        service_block_size = len(service_block) // 2
        total_size = current_size + service_block_size
        
        if total_size <= 28:  # Standard advertising limit
            advertising_data += service_block
        
        return advertising_data
    
    def check_events(self):
        """Check for BLE events (non-blocking) - optimized for high frequency calls"""
        # Handle pending advertising resume with minimal delay
        if self.need_resume_advertising:
            self.send_at_command_fast("AT+UBTDM=3")
            self.need_resume_advertising = False
            
        if not self.uart or not self.uart.any():
            return None
            
        try:
            event = self.uart.read().decode('utf-8', 'ignore').strip()
            if not event:
                return None
                
            # Fast event processing (minimal logging for speed)
            # Connection event
            if "+UUBTACLC" in event:
                parts = event.split(',')
                if len(parts) >= 2:
                    self.connection_handle = parts[1]
                    self.is_connected = True
            
            # Disconnection event
            elif "+UUBTACLD" in event:
                self.is_connected = False
                self.connection_handle = None
                # Set flag to resume advertising
                if self.is_advertising:
                    self.need_resume_advertising = True
                
            return event
                
        except Exception:
            # Silent error handling for speed
            pass
            
        return None
    
    def get_status(self):
        """Get current beacon status"""
        return {
            "initialized": self.is_initialized,
            "advertising": self.is_advertising,
            "connected": self.is_connected,
            "device_name": self.device_name,
            "current_message": self.current_message,
            "connection_handle": self.connection_handle
        }
    
    def close(self):
        """Clean shutdown of the beacon"""
        if self.is_advertising:
            self.stop_advertising()
            
        if self.uart:
            self.uart.close()
            self.uart = None
            
        self.is_initialized = False
        print("🔌 BLE beacon closed")

    def factory_reset(self):
        """Complete factory reset - stops all services and resets to default settings"""
        print("=== FACTORY RESET INITIATED ===")
        
        try:
            # Step 1: Stop all advertising and services
            print("1. Stopping all advertising and services...")
            self.send_at_command("AT+UBTDM=0")  # Stop advertising
            self.send_at_command("AT+UBTGSER=0")  # Stop GATT server if running
            
            # Step 2: Reset all BLE parameters to factory defaults
            print("2. Resetting BLE parameters to factory defaults...")
            
            # Reset advertising parameters with HIGH INTERVAL (5 seconds)
            self.send_at_command("AT+UBTLECFG=1,8000")  # Reset min interval to 5 seconds (8000 units)
            self.send_at_command("AT+UBTLECFG=2,8000")  # Reset max interval to 5 seconds (8000 units)
            self.send_at_command("AT+UBTLECFG=3,7")    # Reset advertising channels
            self.send_at_command("AT+UBTLECFG=4,6")    # Reset connection interval min
            self.send_at_command("AT+UBTLECFG=5,6")    # Reset connection interval max
            self.send_at_command("AT+UBTLECFG=6,0")    # Reset slave latency
            self.send_at_command("AT+UBTLECFG=7,100")  # Reset supervision timeout
            self.send_at_command("AT+UBTLECFG=25,0")   # Reset BR/EDR flag
            self.send_at_command("AT+UBTLECFG=26,0")   # Reset Data Length Extension
            
            # Step 3: Reset device name to default
            print("3. Resetting device name to default...")
            self.send_at_command('AT+UBTLN="ANNA-B4"')  # Reset to default name
            
            # Step 4: Reset connection mode
            print("4. Resetting connection mode...")
            self.send_at_command("AT+UBTCM=0")  # Reset to non-connectable mode
            
            # Step 5: Clear all stored data and settings
            print("5. Clearing stored data and settings...")
            self.send_at_command("AT+UBTAD=")  # Clear advertising data
            
            # Step 6: Save settings before module reset
            print("6. Saving settings...")
            self.send_at_command("AT+UBTSAVE")  # Save all settings to flash
            
            # Step 7: Perform module reset
            print("7. Performing module reset...")
            self.send_at_command("AT+CFUN=1,1")  # Full reset (if supported)
            
            # Step 8: Reset internal state
            print("8. Resetting internal state...")
            self.is_advertising = False
            self.is_connected = False
            self.connection_handle = None
            self.is_initialized = False
            self.need_resume_advertising = False
            self.current_message = ""
            self.device_name = "ANNA-B4"  # Reset to default
            self.use_extended_advertising = False
            self.message_in_device_name = False
            self.max_data_length = 28
            
            print("✅ FACTORY RESET COMPLETED")
            print("📱 Device name reset to: 'ANNA-B4'")
            print("📡 All services stopped")
            print("⚙️ All parameters reset to factory defaults with HIGH INTERVAL (5s)")
            print("💾 Settings saved before reset")
            print("🔄 Module reset performed")
            
            return True
            
        except Exception as e:
            print("❌ Factory reset failed: " + str(e))
            return False

    def test_advertising_limits(self):
        """Test to find the actual advertising data limits"""
        if not self.is_initialized:
            print("Beacon not initialized")
            return
            
        print("\nTesting advertising data limits...")
        print("Data Length Extension: " + str(self.use_extended_advertising))
        
        # Test progressively larger messages with more granular steps
        test_sizes = [5, 8, 10, 12, 15, 18, 20, 22, 25, 28, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 120, 150, 200]
        max_working_size = 0
        
        print("\nTesting message sizes...")
        for size in test_sizes:
            test_message = "X" * size  # Simple repeated character
            print("Testing " + str(size) + " bytes...", end=" ")
            
            try:
                adv_data = self.create_advertising_data(test_message, debug=True)  # Enable debug for testing
                result = self.send_at_command("AT+UBTAD=" + adv_data)
                
                if "OK" in result:
                    max_working_size = size
                    print("Success")
                else:
                    print("FAILED")
                    break
                    
            except Exception as e:
                print("ERROR - " + str(e))
                break
        
        print("\nTest Results:")
        print("  Maximum working message size: " + str(max_working_size) + " bytes")
        print("  Data Length Extension enabled: " + str(self.use_extended_advertising))
        print("  Configured limit: " + str(self.max_data_length) + " bytes")
        
        # Calculate advertising packet breakdown
        if max_working_size > 0:
            test_msg = "X" * max_working_size
            adv_data = self.create_advertising_data(test_msg)
            total_packet_size = len(adv_data) // 2
            print("  Total advertising packet: " + str(total_packet_size) + " bytes")
            print("  Header overhead: " + str(total_packet_size - max_working_size) + " bytes")
            
            # Be conservative - use 90% of max working size
            practical_limit = int(max_working_size * 0.9)
            print("  Recommended practical limit: " + str(practical_limit) + " bytes")
            
        return max_working_size

class BLEGATTServer:
    """High-speed BLE GATT server for IMU data streaming via notifications"""
    
    def __init__(self, uart_port=UART.UART1, baud_rate=115200, device_name="IMU_DRONE_FAST"):
        self.uart_port = uart_port
        self.baud_rate = baud_rate
        self.uart = None
        self.device_name = device_name
        self.is_connected = False
        self.connection_handle = None
        self.is_initialized = False
        self.notifications_enabled = False
        
        # GATT Service and Characteristic UUIDs
        self.service_uuid = "12345678-1234-1234-1234-123456789ABC"  # Custom IMU service
        self.char_uuid = "87654321-4321-4321-4321-CBA987654321"     # IMU data characteristic
        
    def initialize(self):
        """Initialize UART and setup GATT server"""
        if self.is_initialized:
            return True
            
        try:
            self.uart = UART(self.uart_port, self.baud_rate, 8, 0, 1, 0)
            
            print("=== Initializing High-Speed BLE GATT Server ===")
            
            # Basic setup
            self.send_at_command("AT")
            self.send_at_command("ATE0")
            
            # Set device name
            self.send_at_command('AT+UBTLN="' + self.device_name + '"')
            
            # Configure for GATT server mode
            self.send_at_command("AT+UBTCM=2")  # Connectable mode
            
            # Set connection parameters for high speed (7.5ms intervals)
            self.send_at_command("AT+UBTLECFG=4,6")   # Min connection interval (7.5ms)
            self.send_at_command("AT+UBTLECFG=5,6")   # Max connection interval (7.5ms)  
            self.send_at_command("AT+UBTLECFG=6,0")   # Slave latency (0 for lowest latency)
            self.send_at_command("AT+UBTLECFG=7,100") # Supervision timeout
            
            # Enable Data Length Extension for larger packets
            self.send_at_command("AT+UBTLECFG=26,1")
            
            # Configure GATT service
            self.setup_gatt_service()
            
            self.is_initialized = True
            print("🚀 High-speed BLE GATT server initialized")
            print("📡 Max data rate: ~100 Hz (vs 10 Hz advertising)")
            print("📦 Max packet size: 244 bytes (vs 18 bytes advertising)")
            print("⚡ Latency: 7.5ms (vs 100ms+ advertising)")
            
            return True
            
        except Exception as e:
            print("GATT server initialization failed: " + str(e))
            return False
    
    def setup_gatt_service(self):
        """Setup custom GATT service for IMU data"""
        print("Setting up GATT service...")
        
        # Create custom service for IMU data
        # Note: Actual GATT service creation depends on your BLE module's AT commands
        # This is a template - you may need to adjust for your specific module
        
        # Start GATT server
        result = self.send_at_command("AT+UBTGSER=1")  # Enable GATT server
        if "ERROR" in result:
            print("Warning: GATT server command not supported, using basic setup")
        
        print("✅ GATT service configured")
    
    def start_server(self):
        """Start the GATT server and begin advertising"""
        if not self.is_initialized:
            if not self.initialize():
                return False
        
        print("=== Starting High-Speed BLE GATT Server ===")
        
        # Start advertising (for discovery only)
        result = self.send_at_command("AT+UBTDM=3")  # Start connectable advertising
        
        if "OK" in result:
            print("🔍 GATT server advertising (waiting for connection)")
            print("📱 Device name: '" + self.device_name + "'")
            print("💡 Connect with BLE app to start high-speed streaming")
            return True
        else:
            print("❌ Failed to start GATT server")
            return False
    
    def send_imu_data(self, imu_data):
        """Send IMU data via GATT notification (much faster than advertising)"""
        if not self.is_connected:
            return False
            
        if not self.notifications_enabled:
            return False
            
        try:
            # Send data as GATT notification
            # Format: larger packet with more precision and metadata
            timestamp = time.time()
            
            # Enhanced data format with timestamp and more precision
            enhanced_data = "IMU," + "{:.3f}".format(timestamp) + "," + imu_data + "," + str(len(imu_data))
            
            # Convert to hex for AT command (MicroPython compatible)
            data_bytes = enhanced_data.encode()
            data_hex = ""
            for byte in data_bytes:
                data_hex += "{:02x}".format(byte)
            
            # Send GATT notification (adjust command for your BLE module)
            result = self.send_at_command_fast("AT+UBTGNTF=" + str(self.connection_handle) + "," + data_hex)
            
            return "OK" in result
            
        except Exception as e:
            print("GATT notification failed: " + str(e))
            return False
    
    def send_at_command(self, command, timeout=2):
        """Send AT command"""
        if not self.uart:
            return "ERROR"
            
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.1)
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response:
                        break
            time.sleep(0.01)
        
        response_str = response.decode().strip()
        
        # Handle connection events
        if "+UUBTACLC" in response_str:
            self.is_connected = True
            self.connection_handle = "0"  # Simplified
            self.notifications_enabled = True
            print("🔗 Client connected - high-speed streaming active!")
        elif "+UUBTACLD" in response_str:
            self.is_connected = False
            self.notifications_enabled = False
            print("📡 Client disconnected - back to advertising mode")
        
        return response_str
    
    def send_at_command_fast(self, command, timeout=0.1):
        """Ultra-fast AT command for notifications"""
        if not self.uart:
            return "ERROR"
            
        self.uart.write((command + '\r\n').encode())
        time.sleep(0.005)
        
        response = b''
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.uart.any():
                data = self.uart.read()
                if data:
                    response += data
                    if b'OK' in response or b'ERROR' in response:
                        break
            time.sleep(0.001)
        
        return response.decode().strip()
    
    def check_events(self):
        """Check for connection events"""
        if not self.uart or not self.uart.any():
            return None
            
        try:
            event = self.uart.read().decode('utf-8', 'ignore').strip()
            if event:
                if "+UUBTACLC" in event:
                    self.is_connected = True
                    self.notifications_enabled = True
                    print("🔗 High-speed connection established!")
                elif "+UUBTACLD" in event:
                    self.is_connected = False
                    self.notifications_enabled = False
                    print("📡 Connection lost - resuming advertising")
                return event
        except:
            pass
        return None
    
    def get_status(self):
        """Get server status"""
        return {
            "initialized": self.is_initialized,
            "connected": self.is_connected,
            "notifications_enabled": self.notifications_enabled,
            "device_name": self.device_name,
            "connection_handle": self.connection_handle
        }
    
    def close(self):
        """Clean shutdown"""
        if self.uart:
            self.send_at_command("AT+UBTDM=0")  # Stop advertising
            self.uart.close()
            self.uart = None
        self.is_initialized = False
        print("🔌 GATT server closed")

    def factory_reset(self):
        """Complete factory reset for GATT server"""
        print("=== GATT SERVER FACTORY RESET INITIATED ===")
        
        try:
            # Step 1: Stop all services
            print("1. Stopping GATT server and advertising...")
            self.send_at_command("AT+UBTDM=0")  # Stop advertising
            self.send_at_command("AT+UBTGSER=0")  # Stop GATT server
            
            # Step 2: Reset all parameters to factory defaults with HIGH INTERVAL
            print("2. Resetting GATT parameters to factory defaults...")
            self.send_at_command("AT+UBTLECFG=1,8000")  # Reset min interval to 5 seconds
            self.send_at_command("AT+UBTLECFG=2,8000")  # Reset max interval to 5 seconds
            self.send_at_command("AT+UBTLECFG=3,7")    # Reset channels
            self.send_at_command("AT+UBTLECFG=4,6")    # Reset connection interval min
            self.send_at_command("AT+UBTLECFG=5,6")    # Reset connection interval max
            self.send_at_command("AT+UBTLECFG=6,0")    # Reset slave latency
            self.send_at_command("AT+UBTLECFG=7,100")  # Reset supervision timeout
            self.send_at_command("AT+UBTLECFG=26,0")   # Reset Data Length Extension
            
            # Step 3: Reset device name
            print("3. Resetting device name to default...")
            self.send_at_command('AT+UBTLN="ANNA-B4"')
            
            # Step 4: Reset connection mode
            print("4. Resetting connection mode...")
            self.send_at_command("AT+UBTCM=0")  # Reset to non-connectable
            
            # Step 5: Clear all data
            print("5. Clearing all stored data...")
            self.send_at_command("AT+UBTAD=")  # Clear advertising data
            
            # Step 6: Save settings before module reset
            print("6. Saving settings...")
            self.send_at_command("AT+UBTSAVE")  # Save all settings to flash
            
            # Step 7: Module reset
            print("7. Performing module reset...")
            self.send_at_command("AT+CFUN=1,1")  # Full reset
            
            # Step 8: Reset internal state
            print("8. Resetting internal state...")
            self.is_connected = False
            self.connection_handle = None
            self.is_initialized = False
            self.notifications_enabled = False
            self.device_name = "ANNA-B4"
            
            print("✅ GATT SERVER FACTORY RESET COMPLETED")
            print("📱 Device name reset to: 'ANNA-B4'")
            print("📡 All GATT services stopped")
            print("⚙️ All parameters reset to factory defaults with HIGH INTERVAL (5s)")
            print("💾 Settings saved before reset")
            print("🔄 Module reset performed")
            
            return True
            
        except Exception as e:
            print("❌ GATT server factory reset failed: " + str(e))
            return False
