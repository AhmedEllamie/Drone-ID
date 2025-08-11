"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Feb, 2025
Description   : GNSS Handler for Quectel modules with:
                - GNSS configuration management
                - Real-time position/data parsing
                - Power state control
                - ISO 8601 time conversion
                - NMEA sentence configuration
Dependencies  : 
                - usr.modules.logging
                - usr.modules.location
                - usr.modules.common
-----------------------------------------------------
"""

from machine import Pin, UART   
from usr.modules.logging import getLogger
from usr.modules.common import option_lock
from usr.modules.location import GNSS, CoordinateSystemConvert, GNSSPower, GNSSExternalUART, NMEAParse
import _thread
import utime

# Get logger instance - will use global log level
log = getLogger(__name__)

class GNSSHandler:
    """! GNSS Handler for managing GPS/GNSS functionality
    
    Provides comprehensive GNSS management including initialization,
    data parsing, power control, and coordinate conversion.
    """
    
    def __init__(self, config_manager):
        """! Initialize GNSS Handler with ConfigManager
        
        @param config_manager Instance of ConfigManager for accessing GNSS configuration
        @throws ValueError If required GNSS configuration parameters are missing
        """
        self._lock = _thread.allocate_lock()
        self._gnss = None
        self._uart = None
        self._parser = NMEAParse()
        self._running = False
        self._fix = False
        self._data = {
            'lat': None,
            'lon': None,
            'alt': None,
            'speed': 0.0,
            'course': 0.0,
            'sats': 0,
            'fix': False,
            'timestamp': None,
            'datestamp': None,
            'lat_dir': None,
            'lng_dir': None
        }
        
        self.config_manager = config_manager
        gnss_config = self.config_manager.get_config('serial', 'gnss')
        
        # Validate config against location.py requirements
        required = ["mode", "port", "baudrate", "power_pin"]
        if not all(k in gnss_config for k in required):
            raise ValueError("Missing GNSS config parameters")
            
        self._gnss_power = GNSSPower(
            PowerPin=int(gnss_config["power_pin"]), 
            StandbyPin=None, 
            BackupPin=None
        )
        self._converter = CoordinateSystemConvert()
        self._transparent_mode = gnss_config.get("transparent_mode", False)
        
        # Register for configuration changes
        self.config_manager.register_callback(self._handle_config_change)
        
        # Initialize using location.py's GNSS class
        self._init_gnss(gnss_config)

    def _handle_config_change(self, section, key, old_value, new_value):
        """! Handle configuration changes
        
        @param section Configuration section that changed
        @param key Configuration key that changed
        @param old_value Previous value
        @param new_value New value
        """
        if section == "serial" and "gnss" in key:
            log.info("GNSS configuration changed, restarting...")
            self.restart()

    def _init_gnss(self, config):
        """! Initialize location.py's GNSS system
        
        @param config Dictionary containing GNSS configuration
        @throws RuntimeError If GNSS initialization fails
        """
        try:
            # Map config to location.py's parameters
            self._gnss = GNSS(
                gps_mode=GNSS.GPS_MODE.internal if config["mode"] == "internal" 
                else GNSS.GPS_MODE.external_uart,
                UARTn=int(config["port"]),
                buadrate=int(config["baudrate"]),
                databits=int(config["databits"]),
                parity=int(config["parity"]),
                stopbits=int(config["stopbits"]),
                flowctl=int(config["flowctl"]),
                PowerPin=int(config["power_pin"]),
                StandbyPin=None,
                BackupPin=None
            )
            
            # Configure UART if external mode
            if config["mode"] == "external":
                self._uart = UART(
                    int(config["port"]),
                    int(config["baudrate"]),
                    int(config["databits"]),
                    int(config["parity"]),
                    int(config["stopbits"]),
                    int(config["flowctl"])
                )
            
            if not self._gnss.start():
                raise RuntimeError("GNSS thread failed to start")
            
            if not self._verify_gnss():
                raise RuntimeError("GNSS verification failed")

            # Configure transparent mode from config
            if config.get("transparent_mode", False):
                self._gnss.set_trans(1)
                log.info("GNSS transparent mode enabled")
            else:
                self._gnss.set_trans(0)
                log.info("GNSS transparent mode disabled")
                
        except Exception as e:
            self.stop()
            raise RuntimeError("GNSS init failed: {0}".format(e))

    def _verify_gnss(self):
        """! Verify GNSS module is responding
        
        @return bool True if GNSS module is responding
        """
        timeout = 30  # 30 second timeout
        start = utime.time()
        
        while utime.time() - start < timeout:
            # Use location.py's read method directly
            raw_data = self._gnss.read()
            if raw_data:
                log.debug("GNSS verification passed - received {0} bytes".format(len(raw_data)))
                return True
            utime.sleep(5)
            
        log.error(" ============== GNSS verification failed ============== ")
        return False

    @option_lock(_thread.allocate_lock())
    def start(self):
        """! Enable GNSS data collection
        
        @return bool True if GNSS monitoring started successfully
        """
        if not self._running:
            self._running = True
            try:
                log.info(" ============== GNSS monitoring Starting... ============== ")
                _thread.start_new_thread(self._update_loop, ())  
                log.debug("Thread ID : {} started".format(_thread.get_ident()))
                return True
            except Exception as e:
                log.error("Failed to start GNSS thread: {}".format(e))
                self._running = False
                return False
        return False

    def _update_loop(self):
        """! Main GNSS data update loop
        
        Continuously reads and processes GNSS data
        """
        thread_id = _thread.get_ident()
        log.debug("========= ENTERING UPDATE LOOP ========= Thread ID: {}".format(thread_id))
        try:
            while self._running:
                with self._lock:
                    if not self._running:
                        log.debug("Thread {} stopping - running flag is False".format(thread_id))
                        break
                    
                #log.debug("[Thread {}] Loop cycle start".format(thread_id))
                
                try:
                    # Add raw read verification
                    raw_data = self._gnss.read()
                    
                    if not raw_data:
                        log.warning("[Thread {}] |=====> Empty GNSS response".format(thread_id))
                        utime.sleep(1)
                        continue
                    
                    #log.debug("[Thread {}] Raw GNSS data: {}".format(thread_id, raw_data))
                    
                    # Process data with safe conversions
                    processed_data = {
                        'lat': self._safe_float(raw_data.get('lat')),
                        'lon': self._safe_float(raw_data.get('lng')),
                        'alt': self._safe_float(raw_data.get('altitude')),
                        'speed': self._safe_float(raw_data.get('speed'), 0.0),
                        'course': self._safe_float(raw_data.get('course'), 0.0),
                        'sats': self._safe_int(raw_data.get('satellites'), 0),
                        'fix': True if raw_data.get('state') == 'A' else False,
                        'timestamp': raw_data.get('timestamp', ''),
                        'datestamp': raw_data.get('datestamp', ''),
                        'lat_dir': raw_data.get('lat_dir', ''),
                        'lng_dir': raw_data.get('lng_dir', '')
                    }
                    
                    with self._lock:
                        self._data.update(processed_data)
                        self._fix = processed_data['fix']
                        #log.debug("[Thread {}] Processed data: {}".format(thread_id, processed_data))
                        
                    if not self._fix:
                        log.warning("[Thread {}] No GNSS fix - valid data: {}".format(thread_id, processed_data))
                        utime.sleep(10)
                    else:
                        #log.debug("[Thread {}] Valid fix obtained".format(thread_id))
                        utime.sleep(1)  
                    
                except ValueError as ve:
                    log.warning("[Thread {}] Data conversion error: {}".format(thread_id, ve))
                    utime.sleep(1)
                except Exception as e:
                    log.error("[Thread {}] Update error: {}".format(thread_id, e))
                    utime.sleep(1)
                
        except Exception as fatal_error:
            log.critical("[Thread {}] Fatal loop error: {}".format(thread_id, fatal_error))
        finally:
            log.debug("========= EXITING UPDATE LOOP ========= Thread ID: {}".format(thread_id))
            self._running = False

    def _safe_float(self, value, default=None):
        """! Safely convert value to float
        
        @param value Value to convert
        @param default Default value if conversion fails
        @return float Converted value or default
        """
        try:
            if value in (None, "", "None", "*"):
                return default
            return float(value)
        except (TypeError, ValueError):
            return default

    def _safe_int(self, value, default=0):
        """! Safely convert value to integer
        
        @param value Value to convert
        @param default Default value if conversion fails
        @return int Converted value or default
        """
        try:
            if value in (None, "", "None", "*"):
                return default
            return int(float(value))  # Convert through float for decimal strings
        except (TypeError, ValueError):
            return default

    ##################### Public API ####################
    def get_data(self):
        """! Get complete GNSS data
        
        @return dict Copy of current GNSS data
        """
        with self._lock:
            return self._data.copy()  # Return a copy to prevent external modification
    
    def get_latitude(self):
        """! Get current latitude
        
        @return float Current latitude or None
        """
        with self._lock:
            return self._data.get("lat")

    def get_longitude(self):
        """! Get current longitude
        
        @return float Current longitude or None
        """
        with self._lock:
            return self._data.get("lon")

    def get_altitude(self):
        """! Get current altitude
        
        @return float Current altitude or None
        """
        with self._lock:
            return self._data.get("alt")

    def get_speed(self):
        """! Get current speed in km/h
        
        @return float Current speed or None if no fix
        """
        if not self._fix:
            log.warning("Speed requested without valid fix")
            return None
        with self._lock:
            speed = self._data.get("speed")
            log.debug("Speed retrieved: {} km/h".format(speed))
            return speed

    def get_course(self):
        """! Get current heading course in degrees
        
        @return float Current course or None if no fix
        """
        if not self._fix:
            log.warning("Course requested without valid fix")
            return None
        with self._lock:
            course = self._data.get("course")
            log.debug("Course retrieved: {}°".format(course))
            return course

    def get_satellites(self):
        """! Get number of satellites in view
        
        @return int Number of satellites
        """
        with self._lock:
            return self._data.get("sats", 0)

    def stop(self):
        """! Stop GNSS operations
        
        Stops the GNSS thread and powers down the module
        """
        with self._lock:
            self._running = False
        if self._gnss:
            self._gnss.stop()
            log.info(" ============== GNSS stopped ============== ")

    def restart(self):
        """! Restart GNSS with current configuration
        
        Stops and restarts the GNSS module with fresh configuration
        """
        self.stop()
        utime.sleep(5)
        
        # Get fresh configuration
        gnss_config = self.config_manager.get_config('serial', 'gnss')
        self._init_gnss(gnss_config)
        
        utime.sleep(5)
        self.start()
        
    def power_off(self):
        """! Power off GNSS module
        
        @return bool Success status
        """
        status = self._gnss_power.power(0)
        utime.sleep(5)
        log.debug("GNSS Power OFF: {}".format(status))
        return status
    
    def power_on(self):
        """! Power on GNSS module
        
        @return bool Success status
        """
        status = self._gnss_power.power(1)
        utime.sleep(5)
        log.debug("GNSS Power ON: {}".format(status))
        return status

    def get_iso_datetime(self):
        """! Get ISO 8601 formatted datetime
        
        Combines date and time components with validation
        @return str ISO 8601 datetime string with fallbacks
        """
        try:
            iso_date = self.get_iso_date()
            iso_time = self.get_iso_time()
            
            # Basic validation of components
            if not isinstance(iso_date, str) or len(iso_date) != 10:
                iso_date = self._get_fallback_utc_date()
                
            if not isinstance(iso_time, str) or len(iso_time) != 12:
                iso_time = self._get_fallback_utc_time()
            
            # Combine validated components
            return "{}T{}".format(iso_date, iso_time)
            
        except Exception as e:
            log.error("Datetime composition error: {}".format(e))
            return self._get_fallback_utc_datetime()

    def _get_fallback_utc_time(self):
        """! Get fallback UTC time when GNSS time is unavailable
        
        @return str Formatted time string
        """
        t = utime.localtime()  # Use localtime instead of gmtime
        return "{:02d}:{:02d}:{:02d}.000Z".format(t[3], t[4], t[5])

    def _get_fallback_utc_date(self):
        """! Get fallback UTC date when GNSS date is unavailable
        
        @return str Formatted date string
        """
        t = utime.localtime()
        return "{:04d}-{:02d}-{:02d}".format(t[0], t[1], t[2])

    def _get_fallback_utc_datetime(self):
        """! Get combined fallback UTC datetime
        
        @return str Formatted datetime string
        """
        return "{}T{}".format(
            self._get_fallback_utc_date(),
            self._get_fallback_utc_time()
        )

    def get_iso_date(self):
        """! Convert GNSS datestamp to ISO 8601 date format
        
        @return str ISO date format (YYYY-MM-DD) or None
        """
        try:
            with self._lock:
                date_str = self._data.get('datestamp')
                
            if not date_str or len(date_str) != 6:
                return None
                
            # Extract components
            day = date_str[0:2]
            month = date_str[2:4]
            year = date_str[4:6]
            
            # Format ISO date (assume 20xx for year)
            return "20{}-{}-{}".format(year, month, day)
            
        except (ValueError, IndexError, TypeError) as e:
            log.error("Invalid date format: {} ({})".format(date_str, e))
            return None
            
    def get_iso_time(self):
        """! Convert GNSS timestamp to ISO 8601 time format
        
        @return str ISO time format (HH:MM:SS.sssZ) or None
        """
        try:
            with self._lock:
                time_str = self._data.get('timestamp')
                
            if not time_str or len(time_str) < 6:
                return None
                
            # Split time and fractional parts
            if '.' in time_str:
                parts = time_str.split('.')
                time_part = parts[0]
                fractional = parts[1]
            else:
                time_part = time_str
                fractional = '000'
                
            # Pad time_part to 6 digits if needed
            time_part = '0' * (6 - len(time_part)) + time_part
            
            # Extract components
            hours = time_part[0:2]
            minutes = time_part[2:4]
            seconds = time_part[4:6]
            
            # Ensure fractional is exactly 3 digits
            fractional = fractional + '0' * (3 - len(fractional))  # Pad with zeros if too short
            fractional = fractional[0:3]  # Truncate if too long
            
            # Format ISO time
            return "{:02d}:{:02d}:{:02d}.{}Z".format(
                int(hours),
                int(minutes),
                int(seconds),
                fractional
            )
        except (ValueError, IndexError, TypeError) as e:
            log.error("Invalid time format: {} ({})".format(time_str, e))
            return None

    #########################################################
    ############## GNSS Configuration Commands ##############
    #########################################################
    
    def calulate_checksum(self, data):
        """! Calculate checksum for NMEA sentences
        
        @param data NMEA sentence data
        @return str Hexadecimal checksum
        """
        checksum = 0
        for char in data:
            checksum ^= ord(char)
        return "{:02X}".format(checksum)
    
    def send_command(self, command):
        """! Send command to GNSS module
        
        @param command NMEA command to send
        """
        try:
            # Remove any existing '$' or '*' if present
            command = command.strip('$').split('*')[0]
            
            # Compute checksum
            full_command = "$" + command + "*" + self.calulate_checksum(command) + "\r\n"
            log.debug("Sending GNSS Config command: {}".format(full_command))
            
            # Send command
            self._uart.write(full_command.encode())
            utime.sleep(0.1)  
        except Exception as e:
            log.error("Error sending command: {}".format(e))
            pass
        
    def tat_default_gnss_config(self):
        """! Configure GPS to output only GNRMC, GNGGA, and GNGSA
        
        Disables all other NMEA sentences and sets fix rate to 1000ms
        """
        try:
            # Disable all NMEA sentences 
            # self.send_command("PAIR062,-1,0")  # Disable all messages

            # Explicitly enable only the needed messages
            self.send_command("PAIR062,0,1")  # Enable GGA (0)
            self.send_command("PAIR062,2,1")  # Enable GSA (2)
            self.send_command("PAIR062,4,1")  # Enable RMC (4)
            

            #  disable un needed messages
            self.send_command("PAIR062,1,0")  # Disable GLL (1)
            self.send_command("PAIR062,3,0")  # Disable GSV (3)
            self.send_command("PAIR062,5,0")  # Disable VTG (5)
            
            #set gps fix rate to 1000ms
            self.send_command("PAIR050,1000")
            self.send_command("PAIR001,513,0")  # Disable proprietary Quectel messages
            self.send_command("PAIR001,050,0")  # Disable debug info
            
            # Ensure settings are saved
            self.send_command("PAIR513")

            log.debug("GNSS configured to output only GNRMC, GNGGA, and GNGSA WITH 200ms DATA RATE")
        except Exception as e:
            log.error("Error configuring GNSS: {}".format(e))
            pass
