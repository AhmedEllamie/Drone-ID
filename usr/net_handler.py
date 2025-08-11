"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Feb, 2025
Description   : Cellular network management with:
                - Connection monitoring & recovery
                - SMS-based configuration
                - Signal quality analysis
                - Network status callbacks
Dependencies  : 
                - usr.modules.logging
                - usr.modules.net_manage
                - sms
                - usocket
                - uping
-----------------------------------------------------
"""
from usr.modules.logging import getLogger
from usr.modules.net_manage import NetManager
import sms
import _thread
import utime
import uping
import modem

logger = getLogger(__name__)


class NetworkHandler:
    """! Streamlined handler for cellular network connectivity
    
    Provides robust cellular network management with automatic reconnection,
    internet validation, and SMS-based configuration capabilities.
    """
    
    def __init__(self, config_manager):
        """! Initialize with ConfigManager and set up NetManager delegate
        
        @param config_manager Instance of ConfigManager for accessing network configuration
        """
        logger.info("Initializing Network components")
        self.config_manager = config_manager
        self._thread_lock = _thread.allocate_lock()
        self._device_imei = modem.getDevImei()
        self._internet_available = False
        
        # Initialize NetManager as our primary network management delegate
        self.net_manager = NetManager()
        
        # Set callback directly on NetManager to receive network state changes
        self.net_manager.set_callback(self._network_state_callback)
        
        # External callback list for application components
        self.network_callbacks = []
        
        # Register for configuration changes
        self.config_manager.register_callback(self._handle_config_change)
        if not self.config_manager.get_device_info('imei'):
            self.config_manager.update_device_info('imei', self._device_imei)
            
        logger.debug("NetworkHandler initialization complete")
        
    def _handle_config_change(self, section, key, old_value, new_value):
        """! Handle configuration changes
        
        @param section Configuration section that changed
        @param key Configuration key that changed
        @param old_value Previous value
        @param new_value New value
        """
        if section == "network":
            if key == "apn":
                logger.info("APN configuration changed, network restart required")
                self.restart()
                
    def _network_state_callback(self, args):
        """! Callback for network state changes from NetManager
        
        @param args Tuple containing (profile_id, state, additional_info)
        """
        profile_id, state, additional_info = args
        logger.debug("Network state change: profile={}, state={}, info={}".format(
            profile_id, state, additional_info))
        
        if state == 0:  # Disconnected
            logger.warning("Network disconnection detected")
            with self._thread_lock:
                self._internet_available = False
            
            # Validate internet after brief delay (in case of transient disconnection)
            utime.sleep(2)
            self._validate_internet_connection()
        else:  # Connected
            logger.info("Network connection detected")
            # Validate internet connectivity when connection established
            self._validate_internet_connection()
        
        # Notify application callbacks about state change
        self._notify_callbacks(state == 1, "CONNECTED" if state == 1 else "DISCONNECTED")
            
    def start(self):
        """! Initialize network with SIM retries
        
        @return bool Success status
        """
        logger.info("Starting network connection")
        
        # Check SIM first
        if not self._check_sim():
            logger.error("SIM not ready, aborting network start")
            return False
            
        # NetManager.net_connect already implements retries and proper connection flow
        result = self.net_manager.net_connect()
        if result != 0:
            logger.error("Network connection failed with code: {}".format(result))
            return False
            
        # Validate internet connectivity
        self._validate_internet_connection()
        
        # Initialize SMS if needed

        
        # Update IP address if not already set
        if not self.config_manager.get_device_info('ip_address'):
            self.config_manager.update_device_info('ip_address', self.get_ip())
        
        logger.info("Network initialization successful")
        return True
    
    def _check_sim(self):
        """! Check SIM status with configured retries
        
        @return bool True if SIM is ready
        """
        retries = self.config_manager.get_config('network', 'sim_check_retries', 3)
        
        for attempt in range(retries):
            logger.debug("SIM check attempt {}/{}".format(attempt+1, retries))
            sim_status = self.net_manager.sim_status()
            
            if sim_status == 1:
                logger.debug("SIM check passed")
                return True
                
            logger.debug("SIM not ready, waiting before retry...")
            utime.sleep(2)
        
        logger.error("SIM check failed after {} attempts".format(retries))
        return False
    
    def _validate_internet_connection(self):
        """! Validate internet connectivity using configured method
        
        @return bool True if internet is available
        """
        logger.debug("Validating internet connection")
        validation_level = self.config_manager.get_config('network', 'validation_level', 'standard')
        
        # Start with optimistic assumption for minimal validation
        if validation_level == 'minimal':
            with self._thread_lock:
                self._internet_available = self.net_manager.net_status()
            return self._internet_available
            
                
        # Try ping for comprehensive validation
        if validation_level == 'standard':
            try:
                ping_server = self.config_manager.get_config('network', 'ping_server', '8.8.8.8')
                result = uping.ping(ping_server, count=1, timeout=5000, size=32)
                if result and result[0] > 0:
                    logger.debug("Ping successful")
                    with self._thread_lock:
                        self._internet_available = True
                    return True
            except Exception as e:
                logger.debug("Ping failed: {}".format(e))
        
        # If all validation attempts failed
        with self._thread_lock:
            self._internet_available = False
        return False
    
    def register_callback(self, callback):
        """! Register application callback for network state changes
        
        @param callback Function to call on network state changes
        @return bool Success status
        """
        if callable(callback) and callback not in self.network_callbacks:
            self.network_callbacks.append(callback)
            return True
        return False
    
    def unregister_callback(self, callback):
        """! Unregister application callback
        
        @param callback Function to unregister
        @return bool Success status
        """
        if callback in self.network_callbacks:
            self.network_callbacks.remove(callback)
            return True
        return False
    
    def _notify_callbacks(self, is_connected, reason):
        """! Notify all registered callbacks about network state changes
        
        @param is_connected Boolean indicating connection state
        @param reason Reason for the state change
        """
        for callback in self.network_callbacks[:]:  # Copy to avoid modification during iteration
            try:
                callback(is_connected, reason)
            except Exception as e:
                logger.error("Error in network callback: {}".format(e))
    
    def restart(self):
        """! Restart network connection
        
        @return bool Success status
        """
        logger.info("Restarting network connection")
        
        # Use NetManager's reconnect which properly handles disconnect+connect
        result = self.net_manager.net_reconnect()
        
        # Validate internet after reconnection
        if result:
            self._validate_internet_connection()
            
        return result
    
    def is_connected(self):
        """! Check if network is connected
        
        @return bool Connection status
        """
        # Use NetManager's status check which includes SIM, network, and data call state
        return self.net_manager.net_status()
    
    def is_internet_available(self):
        """! Check if internet is available based on validation
        
        @return bool Internet availability status
        """
        with self._thread_lock:
            return self._internet_available
    
    def get_signal_strength(self):
        """! Get current signal strength
        
        @return int Signal strength (0-31)
        """
        return self.net_manager.signal_csq()
    
    def get_signal_level(self):
        """! Get signal level (0-5)
        
        @return int Signal level
        """
        return self.net_manager.signal_level()
    
    def get_network_mode(self):
        """! Get current network mode
        
        @return int Network mode (2=2G, 3=3G, 4=4G)
        """
        return self.net_manager.net_mode()
    
    def get_ip(self):
        """! Get current IP address
        
        @return str IP address or None
        """
        try:
            info = self.net_manager.call_info()
            if isinstance(info, tuple) and len(info) >= 3:
                return info[2][2]
        except Exception as e:
            logger.error("Error getting IP: {}".format(e))
        return None
    
    def get_status(self):
        """! Thread-safe status report
        
        @return dict Status information including connection state, IP, and signal
        """
        logger.debug("Compiling network status report")

        # Get network status and IP
        net_connected = self.net_manager.net_status()
        ip_address = self.get_ip()

        # Access shared state with thread protection
        with self._thread_lock:
            internet = self._internet_available

        status = {
            "connected": net_connected,
            "ip_address": ip_address,
            "internet": internet,
            "signal": {
                "level": self.net_manager.signal_level(),
                "rssi": self.get_signal_strength()
            }
        }        
        return status

    def get_network_type(self):
        """! Return network type as string (2G/3G/4G)
        
        @return str Network type
        """
        mode = self.net_manager.net_mode()
        
        if mode == 4:
            return "4G"
        elif mode == 3:
            return "3G"
        elif mode == 2:
            return "2G"
        else:
            return "Unknown"

    def get_raw_signal(self):
        """! Get raw signal strength (for compatibility with previous code)
        
        @return int Signal strength
        """
        return self.get_signal_strength()
        
    def get_validated_rssi(self):
        """! Get RSSI with value validation (for compatibility with network_handler.py)
        
        @return int Validated RSSI value
        """
        return self.get_signal_strength()
