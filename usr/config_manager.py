"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Feb, 2025
Description   : Configuration Manager with:
                - Singleton pattern implementation
                - Real-time config updates
                - Thread-safe operations
                - Config validation
                - Change notification callbacks
Dependencies  : 
                - ujson
                - _thread
                - usr.modules.logging
-----------------------------------------------------
"""

import ujson
import _thread
from usr.modules.logging import getLogger
import utime

log = getLogger(__name__)

class ConfigManager:
    """! Configuration Manager for telematics system
    
    Implements singleton pattern for centralized configuration management
    with thread-safe operations and change notification system.
    """
    
    _instance = None
    _initialized = False
    
    def __new__(cls, config_path="/usr/config.json"):
        """! Singleton implementation
        
        @param config_path Path to configuration file
        @return ConfigManager instance
        """
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)
        return cls._instance
        
    def __init__(self, config_path="/usr/config.json"):
        """! Initialize Configuration Manager
        
        @param config_path Path to configuration file
        """
        # Ensure initialization happens only once
        if ConfigManager._initialized:
            return
            
        ConfigManager._initialized = True
        self._config_path = config_path
        self._config = {}
        self._lock = _thread.allocate_lock()
        self._change_callbacks = []
        self._load_config()
        
        # Initialize device info section if not exists
        if 'device_info' not in self._config:
            self._config['device_info'] = {
                'imei': None,
                'ip_address': None,
                'battery_voltage': 0.0,
                'battery_level': 0,
                'signal_csq': 0,
                'fw_version': '4.5.3',
                'last_update': 0, 
                'last_uart_activity': 0
            }
        
        
        
    def _load_config(self):
        """! Load configuration from file
        
        @throws Exception If configuration loading fails
        """
        try:
            with self._lock:
                with open(self._config_path, 'r') as f:
                    self._config = ujson.load(f)
            log.info("Configuration loaded successfully")
        except Exception as e:
            log.error("Failed to load config: {}".format(e))
            raise
            
    def _save_config(self):
        """! Save current configuration to file
        
        @return bool Success status
        """
        try:
            with self._lock:
                with open(self._config_path, 'w') as f:
                    ujson.dump(self._config, f)
            log.info("Configuration saved successfully")
            return True
        except Exception as e:
            log.error("Failed to save config: {}".format(e))
            return False
            
    def get_config(self, section=None, key=None, default=None):
        """! Get configuration value(s)
        
        @param section Main config section (e.g., 'network', 'mqtt')
        @param key Specific key in section
        @param default Default value if not found
        @return Configuration value or section
        """
        with self._lock:
            try:
                if section is None:
                    return self._config.copy()
                    
                if key is None:
                    return self._config.get(section, {}).copy()
                    
                return self._config.get(section, {}).get(key, default)
                
            except Exception as e:
                log.error("Error retrieving config: {}".format(e))
                return default
                
    def update_config(self, section, key, value):
        """! Update specific configuration value
        
        @param section Config section to update
        @param key Key to update
        @param value New value
        @return bool Success status
        """
        with self._lock:
            try:
                # Ensure section exists
                if section not in self._config:
                    self._config[section] = {}
                    
                # Update value
                old_value = self._config[section].get(key)
                self._config[section][key] = value
                
                # Save to file
                if self._save_config():
                    # Notify callbacks of change
                    self._notify_change(section, key, old_value, value)
                    return True
                    
                # Revert on save failure
                self._config[section][key] = old_value
                return False
                
            except Exception as e:
                log.error("Failed to update config: {}".format(e))
                return False
                
    def update_section(self, section, config_dict):
        """! Update entire configuration section
        
        @param section Section to update
        @param config_dict New configuration dictionary
        @return bool Success status
        """
        with self._lock:
            try:
                old_section = self._config.get(section, {}).copy()
                self._config[section] = config_dict
                
                if self._save_config():
                    # Notify about section update
                    self._notify_section_change(section, old_section, config_dict)
                    return True
                    
                # Revert on save failure
                self._config[section] = old_section
                return False
                
            except Exception as e:
                log.error("Failed to update config section: {}".format(e))
                return False
                
    def register_callback(self, callback):
        """! Register callback for configuration changes
        
        @param callback Function with signature: callback(section, key, old_value, new_value)
        @return bool Success status
        """
        if callable(callback):
            self._change_callbacks.append(callback)
            return True
        return False
        
    def _notify_change(self, section, key, old_value, new_value):
        """! Notify all callbacks about config change
        
        @param section Section that changed
        @param key Key that changed
        @param old_value Previous value
        @param new_value New value
        """
        for callback in self._change_callbacks:
            try:
                callback(section, key, old_value, new_value)
            except Exception as e:
                log.error("Callback error: {}".format(e))
                
    def _notify_section_change(self, section, old_section, new_section):
        """! Notify about section-wide changes
        
        @param section Section that changed
        @param old_section Previous section values
        @param new_section New section values
        """
        # Find changed keys
        all_keys = set(old_section.keys()) | set(new_section.keys())
        for key in all_keys:
            old_value = old_section.get(key)
            new_value = new_section.get(key)
            if old_value != new_value:
                self._notify_change(section, key, old_value, new_value)
                
    def validate_config(self, section, config_dict):
        """! Validate configuration before updating
        
        @param section Section to validate
        @param config_dict Configuration to validate
        @return bool Validation result
        """
        # Add validation logic based on section
        try:
            if section == "network":
                return self._validate_network_config(config_dict)
            elif section == "mqtt":
                return self._validate_mqtt_config(config_dict)
            elif section == "serial":
                return self._validate_serial_config(config_dict)
            return True
        except Exception as e:
            log.error("Config validation error: {}".format(e))
            return False
            
    def _validate_network_config(self, config):
        """! Validate network configuration
        
        @param config Network configuration dictionary
        @return bool Validation result
        """
        required = ["apn", "timeout", "retry_count"]
        return all(key in config for key in required)
        
    def _validate_mqtt_config(self, config):
        """! Validate MQTT configuration
        
        @param config MQTT configuration dictionary
        @return bool Validation result
        """
        required_broker = ["host", "port", "username", "password"]
        
        if "broker1" not in config or "broker2" not in config:
            return False
            
        return all(key in config["broker1"] for key in required_broker) and \
               all(key in config["broker2"] for key in required_broker)
               
    def _validate_serial_config(self, config):
        """! Validate serial configuration
        
        @param config Serial configuration dictionary
        @return bool Validation result
        """
        required_uart = ["port", "baudrate", "parity", "stopbits"]
        
        if "esp32" not in config or "gnss" not in config:
            return False
            
        return all(key in config["esp32"] for key in required_uart) and \
               all(key in config["gnss"] for key in required_uart)
               
    def reload_config(self):
        """! Reload configuration from file
        
        @return bool Success status
        """
        try:
            with self._lock:
                old_config = self._config.copy()
                self._load_config()
                
                # Notify about all changes
                for section in set(old_config.keys()) | set(self._config.keys()):
                    old_section = old_config.get(section, {})
                    new_section = self._config.get(section, {})
                    if old_section != new_section:
                        self._notify_section_change(section, old_section, new_section)
                        
            return True
        except Exception as e:
            log.error("Failed to reload config: {}".format(e))
            return False 

    def update_device_info(self, key, value):
        """! Update a specific device information value
        
        @param key Device info parameter name
        @param value New value to set
        @return bool Success status
        """
        if key not in self._config['device_info']:
            log.warning("Invalid device_info key: {}".format(key))
            return False
        
        old_value = self._config['device_info'][key]
        self._config['device_info'][key] = value
        self._config['device_info']['last_update'] = utime.time()
        
        # Notify listeners about the change
        self._notify_change('device_info', key, old_value, value)
        return True

    def get_device_info(self, key=None):
        """! Get device information
        
        @param key Specific device info parameter to get (None for all)
        @return value or dict Specific value or entire device_info dict
        """
        if key is None:
            return self._config['device_info'].copy()  # Return a copy to prevent direct modification
        
        if key not in self._config['device_info']:
            log.warning("Invalid device_info key: {}".format(key))
            return None
        
        return self._config['device_info'][key] 