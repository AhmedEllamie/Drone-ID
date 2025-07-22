# Real-Time Drone Takeoff Detection

A Python system for real-time detection of drone takeoff sequences using IMU (accelerometer) data from a serial port.

## Features

- **Real-time data acquisition** from COM3 serial port
- **Live plotting** of acceleration data (X, Y, Z axes + magnitude)
- **4-step takeoff detection**:
  1. **Zeros** - Drone stationary
  2. **All Ripples** - Motors start (vibrations in all axes)
  3. **Z-axis Big Change** - Takeoff (Z-axis goes high then returns to ripples)
  4. **Stabilization** - All axes return to ripples (flight mode)
- **State machine** for robust pattern detection
- **Fallback to CSV simulation** if serial port unavailable

## Requirements

```bash
pip install pandas matplotlib scipy pyserial numpy
```

## Configuration

Edit the parameters in `imu.py`:

```python
# Serial Configuration
SERIAL_PORT = 'COM3'                 # Change to your COM port
BAUD_RATE = 115200                   # Match your device's baud rate

# Detection Parameters
RIPPLE_WINDOW = 20                   # Window size for ripple detection
RIPPLE_THRESHOLD = 0.005             # Variance threshold for ripples
BUFFER_SIZE = 500                    # Data buffer size
PLOT_WINDOW = 200                    # Samples shown in real-time plot

# Mode Selection
USE_LIVE_DATA = True                 # False = use CSV simulation
```

## Serial Data Format

Your device should send data in CSV format over serial:
```
acc_x,acc_y,acc_z
0.102,0.081,0.096
0.149,0.095,0.168
...
```

## Usage

### Live Data Mode
```bash
python imu.py
```

### CSV Simulation Mode
Set `USE_LIVE_DATA = False` in the code, then:
```bash
python imu.py
```

## Real-Time Display

The system shows:
- **4 real-time plots**: X, Y, Z acceleration + magnitude
- **Status bar**: Current state and detection indices
- **Console output**: Detection events with sample numbers

### Detection States
- **Waiting**: Looking for initial movement
- **Ripples**: Motors started, vibrations detected
- **Takeoff**: Z-axis significant change detected
- **Stabilized**: Return to normal flight ripples

## Troubleshooting

### Serial Connection Issues
1. Check COM port number in Device Manager
2. Ensure correct baud rate
3. Verify device permissions
4. System will fallback to CSV simulation if connection fails

### No Detection
1. Adjust `RIPPLE_THRESHOLD` for your sensor sensitivity
2. Check data format matches expected CSV format
3. Verify sensor axes orientation
4. Monitor console for debug messages

### Performance
- Reduce `PLOT_WINDOW` if plotting is slow
- Increase `BUFFER_SIZE` for longer detection windows
- Adjust animation interval (currently 50ms)

## Example Output

```
Connected to COM3 at 115200 baud
Motor start detected at sample 150
Takeoff detected at sample 420
Stabilization detected at sample 850
``` 