import serial
import csv
import time
from datetime import datetime

# Configuration
SERIAL_PORT = 'COM12'
BAUD_RATE = 115200
DURATION_SEC = 20
OUTPUT_FILE = f'imu_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

def parse_serial_line(line):
    """Parse incoming serial data line"""
    try:
        # Assuming data format: "acc_x,acc_y,acc_z\n"
        parts = line.strip().split(',')
        if len(parts) == 3:
            acc_x = float(parts[0])
            acc_y = float(parts[1])
            acc_z = float(parts[2])
            return acc_x, acc_y, acc_z
    except ValueError:
        pass
    return None

def main():
    print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected successfully!")
        
        # Open CSV file for writing
        with open(OUTPUT_FILE, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['acc_x', 'acc_y', 'acc_z'])
            
            print(f"Recording data for {DURATION_SEC} seconds...")
            print(f"Data will be saved to: {OUTPUT_FILE}")
            
            start_time = time.time()
            sample_count = 0
            
            while time.time() - start_time < DURATION_SEC:
                try:
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8')
                        data = parse_serial_line(line)
                        
                        if data:
                            acc_x, acc_y, acc_z = data
                            
                            # Write to CSV
                            writer.writerow([acc_x, acc_y, acc_z])
                            
                            sample_count += 1
                            
                            # Print every 50 samples to show progress
                            if sample_count % 50 == 0:
                                elapsed = time.time() - start_time
                                remaining = DURATION_SEC - elapsed
                                print(f"Sample {sample_count} | Elapsed: {elapsed:.1f}s | Remaining: {remaining:.1f}s | x={acc_x:.3f}, y={acc_y:.3f}, z={acc_z:.3f}")
                
                except Exception as e:
                    print(f"Error reading data: {e}")
                    continue
            
            print(f"\nRecording complete!")
            print(f"Total samples collected: {sample_count}")
            print(f"Data saved to: {OUTPUT_FILE}")
            
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        print("Please check:")
        print("- Device is connected")
        print("- Port name is correct")
        print("- Port is not being used by another application")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        try:
            ser.close()
            print("Serial port closed.")
        except:
            pass

if __name__ == "__main__":
    main() 