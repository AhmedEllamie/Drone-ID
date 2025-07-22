import serial
import time

print("🔍 Testing data format from COM5...")
print("=" * 50)

try:
    # Connect to COM5
    ser = serial.Serial('COM5', 115200, timeout=1)
    print("✅ Connected to COM5")
    
    print("📡 Waiting for data (first 10 lines):")
    print("-" * 40)
    
    for i in range(10):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            parts = line.split(',')
            print(f"Line {i+1}: '{line}'")
            print(f"   Parts: {len(parts)} values: {parts}")
            print(f"   Expected: 9 values (acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z)")
            
            # Test parsing
            if len(parts) == 9:
                try:
                    values = [float(p) for p in parts]
                    print(f"   ✅ Parsed OK: {values}")
                except ValueError as e:
                    print(f"   ❌ Parse error: {e}")
            else:
                print(f"   ❌ Wrong count: Expected 9, got {len(parts)}")
            print()
        else:
            print(f"Line {i+1}: (empty)")
        
        time.sleep(0.1)
        
    ser.close()
    
except Exception as e:
    print(f"❌ Error: {e}")
    print("💡 Make sure:")
    print("   1. EC200A connected to COM5")
    print("   2. main.py is running")
    print("   3. No other software using COM5") 