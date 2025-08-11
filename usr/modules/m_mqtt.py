"""
-----------------------------------------------------
Author        : Mohamed Maher (eng.mohamed.maher2016@gmail.com)
Date Modified : Mar, 2024
Description   : MQTT Client implementation with:
                - Automatic reconnection mechanism
                - Thread-safe operations
                - Network status monitoring
                - Topic subscription management
                - JSON payload handling
Dependencies  : 
                - umqtt
                - utime
                - log
                - net
                - dataCall
                - ujson
-----------------------------------------------------
"""
import utime
import log
import net
import _thread
import dataCall
from umqtt import MQTTClient
import ujson

mqtt_client = None

# Reclaim the thread resource through the status after calling MQTTClient.disconnect().
TaskEnable = True

mqtt_log = log.getLogger(__name__)



# Encapsulate MQTT so it can support more custom logic.
class MqttClient():
    """! MQTT Client wrapper with enhanced functionality
    
    Provides a robust MQTT client implementation with automatic reconnection,
    network status monitoring, and thread-safe operations.
    """

    # Note: The parameter reconn enables or disables the internal reconnection mechanism. Default value: True (enable).
    # If you need to test or use the external reconnection mechanism, please refer to this example code below. Before testing, set reconn to False, otherwise, the internal reconnection mechanism will be used by default.
    def __init__(self, clientid, server, port, user=None, password=None, keepalive=0, ssl=False, ssl_params={},
                 reconn=True):
        """! Initialize MQTT client with connection parameters
        
        @param clientid Client identifier string
        @param server MQTT broker server address
        @param port MQTT broker port number
        @param user Username for authentication (optional)
        @param password Password for authentication (optional)
        @param keepalive Keepalive interval in seconds (default: 0)
        @param ssl Enable SSL/TLS connection (default: False)
        @param ssl_params SSL/TLS parameters dictionary (default: empty)
        @param reconn Enable internal reconnection mechanism (default: True)
        """
        self.__clientid = clientid
        self.__pw = password
        self.__server = server
        self.__port = port
        self.__uasename = user
        self.__keepalive = keepalive
        self.__ssl = ssl
        self.__ssl_params = ssl_params
        self.topic = None
        self.qos = None
        # Network status flag.
        self.__nw_flag = True
        # Create a mutex.
        self.mp_lock = _thread.allocate_lock()
        # Create a class to initialize the MQTT object.
        self.client = MQTTClient(self.__clientid, self.__server, self.__port, self.__uasename, self.__pw,
                                 keepalive=self.__keepalive, ssl=self.__ssl, ssl_params=self.__ssl_params,
                                 reconn=reconn)

    def connect(self):
        """! Connect to the MQTT server and register network callback
        
        Establishes connection to the MQTT broker and registers a callback
        for network status changes.
        
        @throws Exception If network callback registration fails
        """
        self.client.connect()
        # Register the callback function of network status. When the network status changes, the function will be called.
        flag = dataCall.setCallback(self.nw_cb)
        if flag != 0:
            # The network callback registration failed.
            raise Exception("Network callback registration failed")

    def set_callback(self, sub_cb):
        """! Set callback function for incoming messages
        
        @param sub_cb Callback function that will be called when messages are received
        """
        self.client.set_callback(sub_cb)

    def error_register_cb(self, func):
        """! Register callback for MQTT thread errors
        
        @param func Callback function that will be called when MQTT thread errors occur
        """
        self.client.error_register_cb(func)

    def subscribe(self, topic, qos=0):
        """! Subscribe to MQTT topics
        
        @param topic Topic or list of topics to subscribe to
        @param qos Quality of Service level (default: 0)
        """
        self.topic = topic  # Save the topic. Multiple topics can be saved by a list.
        self.qos = qos  # Save the QoS.
        self.client.subscribe(topic, qos)

    def publish(self, topic, msg, qos=0):
        """! Publish message to MQTT topic
        
        Handles automatic conversion of different payload types:
        - Dictionaries and lists are converted to JSON strings
        - Strings are encoded to UTF-8 bytes
        
        @param topic Topic to publish to
        @param msg Message payload (dict, list, str, or bytes)
        @param qos Quality of Service level (default: 0)
        """
        original_type = type(msg)
        if isinstance(msg, (dict, list)):
            msg = ujson.dumps(msg)
        if isinstance(msg, str):
            msg = msg.encode('utf-8')  # Convert to bytes
        mqtt_log.debug("Publish type conversion: {} -> {}, topic: {}".format(
            original_type, type(msg), topic))
        self.client.publish(topic, msg, qos)

    def disconnect(self):
        """! Disconnect from the MQTT server
        
        Stops the message monitoring thread and releases all resources
        by disconnecting from the MQTT server.
        """
        global TaskEnable
        # Close the monitoring thread of wait_msg.
        TaskEnable = False
        # Disconnect from the MQTT server and release the resources.
        self.client.disconnect()

    def reconnect(self):
        """! Implement MQTT reconnection with network validation
        
        Handles reconnection to the MQTT server with proper resource management:
        1. Releases resources from previous connection
        2. Checks network status before attempting reconnection
        3. Resubscribes to previously subscribed topics
        
        @return bool True if reconnection successful
        """
        # Determine whether the lock has been acquired.
        if self.mp_lock.locked():
            return
        self.mp_lock.acquire()
        # Close the previous connection before reconnecting to release resources. Please note the differences between *MQTTClient.disconnect()* and *MQTTClient.close()*, where MQTTClient.close() only releases socket resources but *MQTTClient.disconnect()* releases resources including threads.
        self.client.close()
        # Reconnect to the MQTT server.
        while True:
            net_sta = net.getState()  # Get network registration information.
            if net_sta != -1 and net_sta[1][0] == 1:
                call_state = dataCall.getInfo(1, 0)  # Get data call information.
                if (call_state != -1) and (call_state[2][0] == 1):
                    try:
                        # The network is normal. Reconnect to the MQTT server.
                        self.connect()
                    except Exception as e:
                        # Reconnection to the MQTT server failed. Try again 5 s later.
                        self.client.close()
                        utime.sleep(5)
                        continue
                else:
                    # The network is unrestored. Please wait.
                    utime.sleep(10)
                    continue
                # Connect to the MQTT server successfully and subscribe to the topic.
                try:
                    # Multiple topics can be saved by a list. Traverse the list to resubscribe the topic.
                    if self.topic is not None:
                        self.client.subscribe(self.topic, self.qos)
                    self.mp_lock.release()
                except:
                    # Subscription failed. Reconnect to the MQTT server.
                    self.client.close()
                    utime.sleep(5)
                    continue
            else:
                utime.sleep(5)
                continue
            break  # Stop loop.
        # Exit and reconnect.
        return True

    def nw_cb(self, args):
        """! Network callback function for data call status
        
        @param args Tuple containing network status information
                   where args[1] is the network state (1=connected, 0=disconnected)
        """
        nw_sta = args[1]
        if nw_sta == 1:
            # Network connected.
            mqtt_log.info("*** network connected! ***")
            self.__nw_flag = True
        else:
            # Network disconnected.
            mqtt_log.info("*** network not connected! ***")
            self.__nw_flag = False

    def __listen(self):
        """! Internal message listening thread
        
        Continuously waits for incoming messages and handles network
        disconnections and socket errors with automatic reconnection.
        
        @return int -1 on unrecoverable error
        """
        while True:
            try:
                if not TaskEnable:
                    break
                self.client.wait_msg()
            except OSError as e:
                # Determine whether the network is disconnected.
                if not self.__nw_flag:
                    # Reconnect after the network is restored from disconnection.
                    mqtt_log.warning("|====> Network is disconnected. Reconnecting...")
                    self.reconnect()
                # Reconnect when the socket status is abnormal.
                elif self.client.get_mqttsta() != 0 and TaskEnable:
                    mqtt_log.warning("|====> MQTT socket is abnormal. Reconnecting...")
                    self.reconnect()
                else:
                    # You can call the raise method to return an exception or -1.
                    return -1
                
    def get_mqttStatus(self):
        """! Get current MQTT connection status
        
        @return int Status code:
                   0 = Normal/Connected
                   1 = Connecting
                   2 = Server connection closed
                   Other = Unknown error
        """
        status = self.client.get_mqttsta()
        if status == 0:
            #mqtt_log.debug("MQTT connection Status: normal.")
            pass
        elif status == 1:
            mqtt_log.warning("MQTT connection Status: Connecting")
        elif status == 2:
            mqtt_log.warning("MQTT connection Status: Server connection closed")
        else:
            mqtt_log.error("MQTT connection Status: Unknown")
        return status

    def loop_forever(self):
        """! Start message listening in background thread
        
        Starts a new thread that continuously listens for incoming messages
        and handles reconnection automatically.
        """
        _thread.start_new_thread(self.__listen, ())

    def publish_log(self, message, level="Info", log_topic=None):
        """! Publish a log message to a dedicated MQTT topic
        
        Creates a structured log message with timestamp and publishes
        it to the specified topic.
        
        @param message Log message content
        @param level Log level (default: "Info")
        @param log_topic Topic to publish log to
        """
        try:
            timestamp = "{}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(*utime.localtime())
            log_message = {
                "timestamp": timestamp,
                "level": level,
                "message": message
            }
            payload = ujson.dumps(log_message)
            self.publish(log_topic, payload)
            mqtt_log.info("MQTT log published: {}".format(payload))
        except Exception as e:
            mqtt_log.error("Failed to send MQTT log: {}".format(e))


