"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Feb, 2025
Description   : MQTT Handler for Car Telematics System with:
                - Dual broker failover support
                - Dynamic topic configuration
                - QoS control and reconnection logic
                - Remote configuration via MQTT
                - Thread-safe publishing operations
Dependencies  : 
                - usr.modules.m_mqtt
                - usr.modules.logging
-----------------------------------------------------
"""

import utime
import ujson
import modem
from usr.modules.m_mqtt import MqttClient
from usr.modules.logging import getLogger

log = getLogger(__name__)

class MQTTHandler:
    """! MQTT Handler for managing MQTT connections and messaging
    
    Provides robust MQTT connectivity with automatic broker failover,
    dynamic topic management, and configuration update capabilities.
    """
    
    def __init__(self, config_manager):
        """! Initialize MQTT Handler with ConfigManager
        
        @param config_manager Instance of ConfigManager for accessing MQTT configuration
        """
        self.config_manager = config_manager
        self.current_broker = "broker1"
        self.mqtt_client = None  # Initialize as None, not string
        
        #self.device_imei = modem.getDevImei()
        # to be compatable with tatweer drone system 
        self.device_imei = self.config_manager.get_config("mqtt", "device_imei")
        self.reconnect_attempts = 0
        self.is_mqtt_connected = False
        self.max_reconnect_attempts = self.config_manager.get_config("network", "retry_policy", {}).get("max_retries", 5)
        
        # Setup topics using config
        mqtt_config = self.config_manager.get_config("mqtt")
        self.topics = {
            "data": "{}/drones/{}/data".format(mqtt_config['topics_prefix'], self.device_imei),
            "event": "{}/drones/{}/event".format(mqtt_config['topics_prefix'], self.device_imei),
            "request": "{}/drones/{}/request".format(mqtt_config['topics_prefix'], self.device_imei),
            "logging": "{}/drones/{}/logging".format(mqtt_config['topics_prefix'], self.device_imei),
            "test": "{}/drones/{}/test".format(mqtt_config['topics_prefix'], self.device_imei)
        }
        
        # Register for config changes
        self.config_manager.register_callback(self._handle_config_change)

    def _handle_config_change(self, section, key, old_value, new_value):
        """! Handle configuration changes
        
        @param section Configuration section that changed
        @param key Configuration key that changed
        @param old_value Previous value
        @param new_value New value
        """
        if section == "mqtt":
            if key == "topics_prefix":
                self._update_topics(new_value)
            elif key in ["broker1", "broker2"]:
                if key == self.current_broker:
                    log.info("Active broker configuration changed, reconnecting...")
                    self.reconnect()

    def _update_topics(self, new_prefix):
        """! Update topics with new prefix
        
        @param new_prefix New topic prefix to use
        """
        self.topics = {
            "data": "{}/{}/data".format(new_prefix, self.device_imei),
            "event": "{}/{}/event".format(new_prefix, self.device_imei),
            "request": "{}/{}/request".format(new_prefix, self.device_imei),
            "logging": "{}/{}/logging".format(new_prefix, self.device_imei),
            "test": "{}/{}/test".format(new_prefix, self.device_imei)
        }
        
        # Resubscribe to topics if connected
        if self.mqtt_client and self.mqtt_client.get_connect_state():
            for topic in self.topics.values():
                self.mqtt_client.subscribe(topic)

    def _setup_mqtt_client(self):
        """! Setup MQTT client with current broker configuration
        
        @throws Exception If client setup fails
        """
        try:
            broker_config = self.config_manager.get_config('mqtt', self.current_broker)
            
            # Disconnect existing client if any
            if self.mqtt_client:
                try:
                    self.mqtt_client.disconnect()
                except:
                    pass
                    
            # Create new MqttClient instance
            self.mqtt_client = MqttClient(
                clientid=self.device_imei,
                server=broker_config['host'],
                port=broker_config['port'],
                user=broker_config['username'],
                password=broker_config['password'],
                keepalive=broker_config['keepalive'],
                ssl=broker_config['ssl']
            )
            
            # Set callback for incoming messages
            self.mqtt_client.set_callback(self._on_message)
            log.debug("MQTT client setup successful for broker: {}".format(self.current_broker))
            
        except Exception as e:
            log.error("Failed to setup MQTT client: {}".format(e))
            raise

    def _switch_broker(self):
        """! Switch between primary and backup brokers
        
        Alternates between broker1 and broker2 when connection issues occur
        """
        self.current_broker = "broker2" if self.current_broker == "broker1" else "broker1"
        self.reconnect_attempts = 0
        log.info("Switching to {}".format(self.current_broker))
        self._setup_mqtt_client()

    def connect(self):
        """! Connect to MQTT broker with failover support
        
        @return bool True if connection successful
        """
        while True:
            try:
                if not self.mqtt_client:
                    self._setup_mqtt_client()
                
                log.debug("Attempting connection to {}...".format(self.current_broker))
                self.mqtt_client.connect()
                self.reconnect_attempts = 0
                
                # Subscribe to all topics
                for topic in self.topics.values():
                    self.mqtt_client.subscribe(topic)
                
                log.info("Connected to {}".format(self.current_broker))
                return True
                
            except Exception as e:
                self.reconnect_attempts += 1
                log.error("Connection failed: {}, attempt {}".format(e, self.reconnect_attempts))
                
                if self.reconnect_attempts >= self.max_reconnect_attempts:
                    self._switch_broker()
                    
                # Wait before retry using exponential backoff
                retry_policy = self.config_manager.get_config("network", "retry_policy", {})
                retry_delay = min(
                    retry_policy.get("backoff_factor", 2) ** self.reconnect_attempts,
                    retry_policy.get("max_backoff", 60)
                )
                utime.sleep(retry_delay)

    def reconnect(self):
        """! Force reconnection to current broker
        
        @return bool True if reconnection successful
        """
        try:
            if self.mqtt_client:
                self.mqtt_client.disconnect()
        except:
            pass
        self.mqtt_client = None  # Reset client
        return self.connect()

    def publish_data(self, data, topic_type="data"):
        """! Publish data to specific topic type
        
        @param data Dictionary containing data to publish
        @param topic_type Topic type to publish to (data, event, request, logging, test)
        @throws ValueError If topic_type is invalid
        @throws RuntimeError If MQTT client is not initialized
        """
        try:
            if topic_type not in self.topics:
                raise ValueError("Invalid topic type: {}".format(topic_type))
                
            if not self.mqtt_client:
                raise RuntimeError("MQTT client not initialized")
                
            payload = ujson.dumps(data)
            log.debug("Prepared payload type: {}, sample: {}".format(
                type(payload), str(payload)[:50]))
            broker_config = self.config_manager.get_config('mqtt', self.current_broker)
            
            if self.mqtt_client.get_mqttStatus() != 0:
                log.warning("Client disconnected, attempting reconnection...")
                self.is_mqtt_connected = False
                self._handle_publish_failure()
                self.reconnect()
            else:
                self.is_mqtt_connected = True
                
            self.mqtt_client.publish(self.topics[topic_type], payload, qos=broker_config['qos'])
            log.debug("Published to {}: {}".format(topic_type, payload))
            
        except Exception as e:
            log.error("Failed to publish to {}: {}".format(topic_type, e))
            self._handle_publish_failure()

    def _on_message(self, topic, msg):
        """! Handle incoming MQTT messages
        
        @param topic Topic the message was received on
        @param msg Message payload
        """
        try:
            topic = topic.decode()
            payload = ujson.loads(msg.decode())
            
            # Log received message
            log.debug("Received message on {}: {}".format(topic, payload))
            
            # Handle configuration updates
            if topic == self.topics["request"]:
                if "config_update" in payload:
                    self._handle_config_update(payload["config_update"])
                else:
                    self._handle_request(payload)
            elif topic == self.topics["data"]:
                self._handle_data(payload)
            
        except Exception as e:
            log.error("Error processing message: {}".format(e))

    def _handle_config_update(self, config_update):
        """! Handle configuration updates received via MQTT
        
        @param config_update Dictionary containing configuration updates
        """
        try:
            for section, updates in config_update.items():
                if isinstance(updates, dict):
                    # Validate before applying
                    if self.config_manager.validate_config(section, updates):
                        self.config_manager.update_section(section, updates)
                        log.info("Configuration section {} updated".format(section))
                    else:
                        log.error("Invalid configuration for section: {}".format(section))
                        
        except Exception as e:
            log.error("Config update failed: {}".format(e))

    def _handle_publish_failure(self):
        """! Handle publishing failures
        
        Increments reconnection attempts and switches broker if needed
        """
        self.reconnect_attempts += 1
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self._switch_broker()
            self.connect()

    def _handle_request(self, payload):
        """! Handle incoming requests
        
        @param payload Request payload
        """
        # TODO: Implement request handling logic
        pass

    def _handle_data(self, payload):
        """! Handle incoming data messages
        
        @param payload Data payload
        """
        # TODO: Implement data handling logic
        pass

    def disconnect(self):
        """! Disconnect from MQTT broker
        
        Safely disconnects from the current broker
        """
        if self.mqtt_client:
            try:
                self.mqtt_client.disconnect()
                log.info("Disconnected from MQTT broker")
            except Exception as e:
                log.error("Error disconnecting: {}".format(e))
            finally:
                self.mqtt_client = None 