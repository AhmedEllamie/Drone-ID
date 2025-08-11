"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Feb, 2025
Description   : Simple IMU Handler for ICM20948 sensor with:
                - Movement detection for sleep wake-up
                - Propeller detection (only when awake)
                - Thread-safe operations

Dependencies  : 
                - usr.modules.logging
                - usr.modules.common
                - machine.I2C
-----------------------------------------------------
"""
import math
import _thread
import utime
from machine import I2C
from usr.modules.logging import getLogger
from usr.modules.common import option_lock

# Get logger instance
log = getLogger(__name__)

# Calibration constants
CALIBRATION_SAMPLES = 100
CALIBRATION_DELAY_MS = 10

class IMUHandler:
    """! Simple IMU Handler for ICM20948 sensor"""
    
    def __init__(self, config_manager):
        """! Initialize IMU Handler with ConfigManager"""
        self._lock = _thread.allocate_lock()
        self._running = False
        self._i2c_obj = None
        self._config_manager = config_manager
        self._event_callback = None
        self._sleep_mode = False  # Track sleep mode state
        
        # IMU sensor configuration
        self.I2C_SLAVE_ADDR = 0x69
        self.REG_ACCEL_XOUT_H = 0x2D
        self.REG_GYRO_XOUT_H = 0x33
        self.REG_MAG_CNTL2 = 0x0A  
        self.REG_MAG_XOUT_L = 0x11  # Magnetometer data registers
        self.REG_MAG_ST2 = 0x18     # Magnetometer status register
        self.PWR_MGMT_1 = 0x06
        self.PWR_MGMT_2 = 0x07
        self.ACCEL_CONFIG = 0x14
        self.GYRO_CONFIG = 0x01
        
        # Initialize sensor values
        self._data = {
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'mag': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'heading': 0.0},
            'propellers_on': False
        }
        
        # Calibration data
        self._calibration = {
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'is_calibrated': False
        }
        
        # Movement detection parameters
        self._last_movement_event_time = 0
        
        # Propeller detection parameters
        self._vibration_buffer = []
        self._propellers_on = False
        self._last_propeller_event_time = 0
        self._propeller_confidence = 0.0
        self._sustained_vibration_start = 0
        self._last_confidence_update = 0
        
        # Load configuration
        self._load_config()
        
        # Initialize I2C and sensor
        self._init_i2c()
        self._init_sensor()
        
        # Register for configuration changes
        self._config_manager.register_callback(self._handle_config_change)
        
    def _load_config(self):
        """! Load IMU configuration from ConfigManager"""
        try:
            imu_cfg = self._config_manager.get_config('imu_cfg')
            if not imu_cfg:
                raise ValueError("imu_cfg section not found in config")
                
            # Get sleep configuration for movement detection
            sleep_cfg = self._config_manager.get_config('sleep_mode')
            
            # Movement detection thresholds
            self.MOVEMENT_THRESHOLD_ACTIVE = sleep_cfg.get('movement_threshold', 0.5) if sleep_cfg else 0.5  # g when device awake
            self.MOVEMENT_THRESHOLD_SLEEP = sleep_cfg.get('wake_sensitivity', 0.15) if sleep_cfg else 0.15     # g when device asleep
            
            self.MOVEMENT_DEBOUNCE = 1.0   # seconds between movement events
            
            log.info("Movement thresholds – active: {:.2f}g  sleep: {:.2f}g".format(
                     self.MOVEMENT_THRESHOLD_ACTIVE, self.MOVEMENT_THRESHOLD_SLEEP))
            
            # Propeller detection parameters
            propeller_cfg = imu_cfg.get('propeller_detection', {})
            self.PROPELLER_DETECTION_ENABLED = propeller_cfg.get('enabled', True)
            sensitivity = propeller_cfg.get('sensitivity', 'medium').lower()
            
            # Set parameters based on sensitivity level
            if sensitivity == 'high':
                self.VIBRATION_THRESHOLD = 0.12
                self.RMS_THRESHOLD = 0.06
                self.CONFIDENCE_THRESHOLD = 0.80
                self.SUSTAINED_DURATION = 4.0
                self.PEAK_COUNT_THRESHOLD = 12
            elif sensitivity == 'low':
                self.VIBRATION_THRESHOLD = 0.20
                self.RMS_THRESHOLD = 0.12
                self.CONFIDENCE_THRESHOLD = 0.90
                self.SUSTAINED_DURATION = 8.0
                self.PEAK_COUNT_THRESHOLD = 20
            else:  # medium (default)
                self.VIBRATION_THRESHOLD = 0.15
                self.RMS_THRESHOLD = 0.08
                self.CONFIDENCE_THRESHOLD = 0.85
                self.SUSTAINED_DURATION = 5.0
                self.PEAK_COUNT_THRESHOLD = 15
            
            # Technical parameters
            self.DETECTION_WINDOW = 50
            self.PROPELLER_DEBOUNCE_TIME = 3.0
            self.CONFIDENCE_DECAY_RATE = 0.1
            
            log.info("IMU configuration loaded successfully")
            
        except Exception as e:
            log.error("Failed to load IMU configuration: {}".format(e))
            raise
            
    def _handle_config_change(self, section, key, old_value, new_value):
        """! Handle configuration changes"""
        if section == 'imu_cfg':
            log.info("IMU configuration changed, reloading...")
            self._load_config()
            
    def _init_i2c(self):
        """! Initialize I2C communication"""
        try:
            self._i2c_obj = I2C(I2C.I2C0, I2C.STANDARD_MODE)
            log.info("I2C initialized successfully")
        except Exception as e:
            log.error("Failed to initialize I2C: {}".format(e))
            raise
            
    def _init_sensor(self):
        """! Initialize the ICM20948 sensor"""
        try:
            # Reset device and set clock source
            self._write_register(self.PWR_MGMT_1, 0x01)
            utime.sleep_ms(100)

            # Accelerometer: Set full scale to ±2g
            self._write_register(self.ACCEL_CONFIG, 0x01)

            # Gyroscope: Set full scale to ±250 dps
            self._write_register(self.GYRO_CONFIG, 0x01)

            # Magnetometer: Enable continuous mode
            self._write_register(self.REG_MAG_CNTL2, 0x16)

            # Ensure sensors are powered on
            self._write_register(self.PWR_MGMT_2, 0x00)

            log.info("ICM20948 initialized successfully")
            
        except Exception as e:
            log.error("Failed to initialize sensor: {}".format(e))
            raise
            
    def _write_register(self, register, value):
        """! Write a value to a specific register"""
        data = bytearray([value])
        reg = bytearray([register])
        self._i2c_obj.write(self.I2C_SLAVE_ADDR, reg, 1, data, 1)

    def _read_register(self, register, length=1):
        """! Read bytes from a specific register"""
        reg = bytearray([register])
        r_data = bytearray(length)
        self._i2c_obj.read(self.I2C_SLAVE_ADDR, reg, 1, r_data, length, 0)
        return r_data
        
    def start(self):
        """! Start IMU data collection"""
        with self._lock:
            if not self._running:
                self._running = True
                try:
                    log.info("IMU monitoring starting...")
                    _thread.start_new_thread(self._update_loop, ())
                    return True
                except Exception as e:
                    log.error("Failed to start IMU thread: {}".format(e))
                    self._running = False
                    return False
            return False
        
    def is_running(self):
        """! Check if the IMU update thread is still running"""
        with self._lock:
            return self._running
        
    def _calibrate_sensor(self):
        """! Calibrate IMU sensor"""
        log.info("Starting sensor calibration. Please keep the sensor still...")
        
        accel_x_sum = 0.0
        accel_y_sum = 0.0
        accel_z_sum = 0.0
        gyro_x_sum = 0.0
        gyro_y_sum = 0.0
        gyro_z_sum = 0.0
        
        # Collect samples for averaging
        for i in range(CALIBRATION_SAMPLES):
            try:
                # Read accelerometer data
                accel_data = self._read_register(self.REG_ACCEL_XOUT_H, 6)
                accel_x = ((accel_data[0] << 8) | accel_data[1])
                accel_y = ((accel_data[2] << 8) | accel_data[3])
                accel_z = ((accel_data[4] << 8) | accel_data[5])
                
                # Convert to g
                accel_x = accel_x if accel_x < 32768 else accel_x - 65536
                accel_y = accel_y if accel_y < 32768 else accel_y - 65536
                accel_z = accel_z if accel_z < 32768 else accel_z - 65536
                
                accel_x_g = accel_x / 16384.0
                accel_y_g = accel_y / 16384.0
                accel_z_g = accel_z / 16384.0
                
                # Read gyroscope data
                gyro_data = self._read_register(self.REG_GYRO_XOUT_H, 6)
                gyro_x = ((gyro_data[0] << 8) | gyro_data[1])
                gyro_y = ((gyro_data[2] << 8) | gyro_data[3])
                gyro_z = ((gyro_data[4] << 8) | gyro_data[5])
                
                # Convert to degrees per second
                gyro_x = gyro_x if gyro_x < 32768 else gyro_x - 65536
                gyro_y = gyro_y if gyro_y < 32768 else gyro_y - 65536
                gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536
                
                gyro_x = gyro_x * 250.0 / 32768.0
                gyro_y = gyro_y * 250.0 / 32768.0
                gyro_z = gyro_z * 250.0 / 32768.0
                
                # Accumulate values
                accel_x_sum += accel_x_g
                accel_y_sum += accel_y_g
                accel_z_sum += accel_z_g
                gyro_x_sum += gyro_x
                gyro_y_sum += gyro_y
                gyro_z_sum += gyro_z
                
                utime.sleep_ms(CALIBRATION_DELAY_MS)
                
            except Exception as e:
                log.error("Error during calibration: {}".format(e))
                return False
        
        # Calculate average offsets
        with self._lock:
            self._calibration['accel']['x'] = accel_x_sum / CALIBRATION_SAMPLES
            self._calibration['accel']['y'] = accel_y_sum / CALIBRATION_SAMPLES
            self._calibration['accel']['z'] = (accel_z_sum / CALIBRATION_SAMPLES) - 1.0  # Subtract 1g for Z axis
            
            self._calibration['gyro']['x'] = gyro_x_sum / CALIBRATION_SAMPLES
            self._calibration['gyro']['y'] = gyro_y_sum / CALIBRATION_SAMPLES
            self._calibration['gyro']['z'] = gyro_z_sum / CALIBRATION_SAMPLES
            
            self._calibration['is_calibrated'] = True
            
        log.info("Calibration complete")
        log.info("Accelerometer offsets: X={:.3f}, Y={:.3f}, Z={:.3f}".format(
            self._calibration['accel']['x'],
            self._calibration['accel']['y'],
            self._calibration['accel']['z']))
        log.info("Gyroscope offsets: X={:.3f}, Y={:.3f}, Z={:.3f}".format(
            self._calibration['gyro']['x'],
            self._calibration['gyro']['y'],
            self._calibration['gyro']['z']))
            
        return True

    def _update_loop(self):
        """! Main IMU data update loop"""
        try:
            # Perform calibration if not already calibrated
            if not self._calibration['is_calibrated']:
                if not self._calibrate_sensor():
                    log.error("Failed to calibrate sensor")
                    return
                
            last_heartbeat = utime.ticks_ms()  # Heart-beat timer
                    
            while self._running:
                with self._lock:
                    if not self._running:
                        log.warning(" =========IMU update loop thread stopped=========")
                        break
                        
                try:
                    # Read accelerometer data
                    accel_data = self._read_register(self.REG_ACCEL_XOUT_H, 6)
                    accel_x = ((accel_data[0] << 8) | accel_data[1])
                    accel_y = ((accel_data[2] << 8) | accel_data[3])
                    accel_z = ((accel_data[4] << 8) | accel_data[5])
                    
                    # Convert to g
                    accel_x = accel_x if accel_x < 32768 else accel_x - 65536
                    accel_y = accel_y if accel_y < 32768 else accel_y - 65536
                    accel_z = accel_z if accel_z < 32768 else accel_z - 65536
                    
                    accel_x_g = accel_x / 16384.0
                    accel_y_g = accel_y / 16384.0
                    accel_z_g = accel_z / 16384.0
                    
                    # Read gyroscope data
                    gyro_data = self._read_register(self.REG_GYRO_XOUT_H, 6)
                    gyro_x = ((gyro_data[0] << 8) | gyro_data[1])
                    gyro_y = ((gyro_data[2] << 8) | gyro_data[3])
                    gyro_z = ((gyro_data[4] << 8) | gyro_data[5])
                    
                    # Convert to degrees per second
                    gyro_x = gyro_x if gyro_x < 32768 else gyro_x - 65536
                    gyro_y = gyro_y if gyro_y < 32768 else gyro_y - 65536
                    gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536
                    
                    gyro_x = gyro_x * 250.0 / 32768.0
                    gyro_y = gyro_y * 250.0 / 32768.0
                    gyro_z = gyro_z * 250.0 / 32768.0
                    
                    # Read magnetometer data (if available)
                    try:
                        # Check if magnetometer data is ready
                        mag_status = self._read_register(self.REG_MAG_ST2, 1)
                        if mag_status[0] & 0x01:  # Data ready bit
                            mag_data = self._read_register(self.REG_MAG_XOUT_L, 6)
                            mag_x = ((mag_data[1] << 8) | mag_data[0])
                            mag_y = ((mag_data[3] << 8) | mag_data[2])
                            mag_z = ((mag_data[5] << 8) | mag_data[4])
                            
                            # Convert to signed values
                            mag_x = mag_x if mag_x < 32768 else mag_x - 65536
                            mag_y = mag_y if mag_y < 32768 else mag_y - 65536
                            mag_z = mag_z if mag_z < 32768 else mag_z - 65536
                            
                            # Convert to microtesla (μT)
                            mag_x_ut = mag_x * 0.15  # Scale factor for AK09916
                            mag_y_ut = mag_y * 0.15
                            mag_z_ut = mag_z * 0.15
                        else:
                            # Use previous values if data not ready
                            with self._lock:
                                mag_data_prev = self._data['mag']
                            mag_x_ut = mag_data_prev['x']
                            mag_y_ut = mag_data_prev['y']
                            mag_z_ut = mag_data_prev['z']
                    except Exception as e:
                        # If magnetometer fails, use zero values
                        mag_x_ut = 0.0
                        mag_y_ut = 0.0
                        mag_z_ut = 0.0
                    
                    # Apply calibration if available
                    if self._calibration['is_calibrated']:
                        accel_x_g -= self._calibration['accel']['x']
                        accel_y_g -= self._calibration['accel']['y']
                        accel_z_g -= self._calibration['accel']['z']
                        
                        gyro_x -= self._calibration['gyro']['x']
                        gyro_y -= self._calibration['gyro']['y']
                        gyro_z -= self._calibration['gyro']['z']
                    
                    # Calculate orientation
                    roll = math.atan2(accel_y_g, accel_z_g) * 180 / math.pi
                    pitch = math.atan2(-accel_x_g, math.sqrt(accel_y_g**2 + accel_z_g**2)) * 180 / math.pi
                    heading = math.atan2(accel_y_g, accel_x_g) * 180 / math.pi
                    
                    # Update data with thread safety
                    with self._lock:
                        self._data.update({
                            'accel': {'x': accel_x_g, 'y': accel_y_g, 'z': accel_z_g},
                            'gyro': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                            'mag': {'x': mag_x_ut, 'y': mag_y_ut, 'z': mag_z_ut},
                            'orientation': {'roll': roll, 'pitch': pitch, 'heading': heading}
                        })
                        
                    # Simple movement detection for sleep wake-up
                    self._detect_movement()
                    
                    # Propeller detection only when not in sleep mode
                    if not self._sleep_mode:
                        self._detect_propeller_vibration()
                    
                    # Heart-beat while in sleep mode every 5 s
                    if self._sleep_mode and utime.ticks_diff(utime.ticks_ms(), last_heartbeat) >= 5000:
                        log.debug("IMU heartbeat – still alive inside sleep mode")
                        last_heartbeat = utime.ticks_ms()
                    
                    utime.sleep_ms(100)  # 100ms update rate
                    
                except Exception as e:
                    log.error("Error in IMU update loop: {}".format(e))
                    utime.sleep(1)
                    
        except Exception as e:
            log.error("Fatal error in IMU update loop: {}".format(e))
        finally:
            self._running = False
            
    def _detect_movement(self):
        """! Simple movement detection based on acceleration magnitude"""
        try:
            current_time = utime.time()
            
            # Check debounce time
            if current_time - self._last_movement_event_time < self.MOVEMENT_DEBOUNCE:
                return False
            
            # Get current acceleration
            with self._lock:
                accel_data = self._data['accel']
            
            # Calculate total acceleration magnitude
            accel_magnitude = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
            
            # Compare to 1g (standard gravity) - if significantly different, it's movement
            movement_magnitude = abs(accel_magnitude - 1.0)
            
            # Get appropriate threshold based on sleep mode
            movement_threshold = self.MOVEMENT_THRESHOLD_SLEEP if self._sleep_mode else self.MOVEMENT_THRESHOLD_ACTIVE
            
            # Debug raw magnitude while in sleep mode for diagnostics
            if self._sleep_mode:
                log.debug("Sleep-mode accel magnitude: {:.3f}g (threshold {:.3f}g)".format(
                    movement_magnitude, movement_threshold))
            
            # Check if movement exceeds threshold
            if movement_magnitude > movement_threshold:
                self._last_movement_event_time = current_time
                
                # Call movement event callback
                if self._event_callback:
                    event_data = {
                        'magnitude': movement_magnitude,
                        'timestamp': current_time
                    }
                    self._event_callback("movement", event_data)
                    
                log.debug("Movement detected - magnitude: {:.3f}g (accel_total: {:.3f}g)".format(
                    movement_magnitude, accel_magnitude))
                return True
            else:
                # In sleep mode, add debug info for non-triggering movements
                if self._sleep_mode and movement_magnitude > 0.05:  # Only log significant but sub-threshold movements
                    log.debug("Movement below threshold - magnitude: {:.3f}g (threshold: {:.3f}g, accel_total: {:.3f}g)".format(
                        movement_magnitude, movement_threshold, accel_magnitude))
                
            return False
            
        except Exception as e:
            log.error("Error in movement detection: {}".format(e))
            return False
    
    def _detect_propeller_vibration(self):
        """! Detect propeller ON/OFF state based on vibration analysis"""
        if not self.PROPELLER_DETECTION_ENABLED:
            return False
            
        try:
            current_time = utime.time()
            
            # Add new vibration data to buffer
            with self._lock:
                accel_data = self._data['accel']
                gyro_data = self._data['gyro']
                
            # Calculate vibration magnitude
            accel_magnitude = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
            gyro_magnitude = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)
            
            # Store vibration sample
            vibration_sample = {
                'time': current_time,
                'accel_mag': accel_magnitude,
                'gyro_mag': gyro_magnitude,
                'accel_x': accel_data['x'],
                'accel_y': accel_data['y'],
                'accel_z': accel_data['z']
            }
            
            self._vibration_buffer.append(vibration_sample)
            
            # Keep buffer size limited
            while len(self._vibration_buffer) > self.DETECTION_WINDOW:
                self._vibration_buffer.pop(0)
                
            # Need minimum samples for analysis
            if len(self._vibration_buffer) < self.DETECTION_WINDOW // 2:
                return False
                
            # Analyze vibration patterns
            propeller_detected = self._analyze_vibration_patterns()
                
            # Check for state changes with debounce protection
            if propeller_detected and not self._propellers_on:
                if current_time - self._last_propeller_event_time >= self.PROPELLER_DEBOUNCE_TIME:
                    self._propellers_on = True
                    self._last_propeller_event_time = current_time
                    
                    with self._lock:
                        self._data['propellers_on'] = True
                        
                    log.info("Propellers ON detected (Confidence: {:.2f})".format(self._propeller_confidence))
                    
                    if self._event_callback:
                        event_data = {
                            'propellers_on': True,
                            'confidence': self._propeller_confidence,
                            'timestamp': current_time
                        }
                        self._event_callback("propellers_on", event_data)
                        
            elif not propeller_detected and self._propellers_on:
                if (self._propeller_confidence < self.CONFIDENCE_THRESHOLD * 0.5 and 
                    current_time - self._last_propeller_event_time >= self.PROPELLER_DEBOUNCE_TIME):
                    
                    self._propellers_on = False
                    self._last_propeller_event_time = current_time
                    self._sustained_vibration_start = 0
                    
                    with self._lock:
                        self._data['propellers_on'] = False
                        
                    log.info("Propellers OFF detected (Confidence: {:.2f})".format(self._propeller_confidence))
                    
                    if self._event_callback:
                        event_data = {
                            'propellers_on': False,
                            'confidence': self._propeller_confidence,
                            'timestamp': current_time
                        }
                        self._event_callback("propellers_off", event_data)
                    
            return propeller_detected
            
        except Exception as e:
            log.error("Error in propeller vibration detection: {}".format(e))
            return False
            
    def _analyze_vibration_patterns(self):
        """! Analyze vibration patterns to detect propeller operation"""
        try:
            current_time = utime.time()
            
            if len(self._vibration_buffer) < 20:
                return False
                
            # Extract vibration data
            accel_mags = [sample['accel_mag'] for sample in self._vibration_buffer]
            gyro_mags = [sample['gyro_mag'] for sample in self._vibration_buffer]
            
            # Calculate RMS (Root Mean Square) of vibrations
            accel_rms = math.sqrt(sum(x**2 for x in accel_mags) / len(accel_mags))
            gyro_rms = math.sqrt(sum(x**2 for x in gyro_mags) / len(gyro_mags))
            
            # Calculate standard deviation
            accel_mean = sum(accel_mags) / len(accel_mags)
            accel_std = math.sqrt(sum((x - accel_mean)**2 for x in accel_mags) / len(accel_mags))
            
            # Count vibration peaks
            peak_count = 0
            for i in range(2, len(accel_mags) - 2):
                if (accel_mags[i] > accel_mags[i-1] and 
                    accel_mags[i] > accel_mags[i+1] and 
                    accel_mags[i] > accel_mags[i-2] and
                    accel_mags[i] > accel_mags[i+2] and
                    accel_mags[i] > self.VIBRATION_THRESHOLD):
                    peak_count += 1
            
            # Calculate confidence score
            confidence_score = 0.0
            
            # Factor 1: High RMS vibration level (40% weight)
            if accel_rms > self.RMS_THRESHOLD:
                accel_factor = min(1.0, (accel_rms - self.RMS_THRESHOLD) / self.RMS_THRESHOLD)
                confidence_score += 0.40 * accel_factor
            
            # Factor 2: Gyroscope contribution (30% weight)  
            if gyro_rms > self.RMS_THRESHOLD * 0.5:
                gyro_factor = min(1.0, (gyro_rms - self.RMS_THRESHOLD * 0.5) / (self.RMS_THRESHOLD * 0.5))
                confidence_score += 0.30 * gyro_factor
                
            # Factor 3: Peak count (30% weight)
            if peak_count >= self.PEAK_COUNT_THRESHOLD:
                peak_factor = min(1.0, peak_count / (self.PEAK_COUNT_THRESHOLD * 2))
                confidence_score += 0.30 * peak_factor
            
            # Apply confidence decay
            time_since_update = current_time - self._last_confidence_update
            if time_since_update > 1.0:
                decay_amount = self.CONFIDENCE_DECAY_RATE * time_since_update
                self._propeller_confidence = max(0.0, self._propeller_confidence - decay_amount)
            
            # Update confidence
            if confidence_score > self._propeller_confidence:
                self._propeller_confidence = confidence_score
                self._last_confidence_update = current_time
            
            # Check sustained duration requirement
            sustained_detection = False
            if self._propeller_confidence >= self.CONFIDENCE_THRESHOLD:
                if self._sustained_vibration_start == 0:
                    self._sustained_vibration_start = current_time
                    
                sustained_time = current_time - self._sustained_vibration_start
                sustained_detection = sustained_time >= self.SUSTAINED_DURATION
            else:
                self._sustained_vibration_start = 0
                
            return sustained_detection
            
        except Exception as e:
            log.error("Error in vibration pattern analysis: {}".format(e))
            return False
    
    def set_sleep_mode(self, sleep_mode):
        """! Set sleep mode state"""
        with self._lock:
            self._sleep_mode = sleep_mode
            if sleep_mode:
                log.info("IMU entering sleep mode flag set - propeller detection disabled, movement threshold {:.2f}g".format(self.MOVEMENT_THRESHOLD_SLEEP))
            else:
                log.info("IMU exiting sleep mode flag cleared - propeller detection enabled, movement threshold {:.2f}g".format(self.MOVEMENT_THRESHOLD_ACTIVE))
            
    def stop(self):
        """! Stop IMU operations"""
        with self._lock:
            self._running = False
        log.info("IMU monitoring stopped")
        
    def get_data(self):
        """! Get complete IMU data"""
        with self._lock:
            return self._data.copy()
            
    def get_accel(self):
        """! Get accelerometer data"""
        with self._lock:
            return self._data['accel'].copy()
            
    def get_gyro(self):
        """! Get gyroscope data"""
        with self._lock:
            return self._data['gyro'].copy()
            
    def get_orientation(self):
        """! Get orientation data"""
        with self._lock:
            return self._data['orientation'].copy()

    def get_mag(self):
        """! Get magnetometer data"""
        with self._lock:
            return self._data['mag'].copy()

    def set_event_callback(self, callback):
        """! Set callback function for events"""
        with self._lock:
            self._event_callback = callback
            
    def are_propellers_on(self):
        """! Check if propellers are currently detected as ON"""
        with self._lock:
            return self._data['propellers_on']
            
    def get_propeller_confidence(self):
        """! Get current propeller detection confidence"""
        with self._lock:
            return self._propeller_confidence
            
    def get_propeller_status(self):
        """! Get complete propeller status information"""
        with self._lock:
            return {
                'propellers_on': self._data['propellers_on'],
                'confidence': self._propeller_confidence,
                'last_event_time': self._last_propeller_event_time
            }
