"""
Drone Status Broadcaster - Main File
Imports existing algorithm and BLE beacon to broadcast drone status
"""

import utime
from new_algorithm_final import SineDetectionSystem
from anna_advertising_beacon import BLEAdvertisingBeacon


class DroneStatusBroadcaster:
    """Main system that integrates existing detection with BLE broadcasting"""
    
    def __init__(self, config_manager):
        # Use existing detection system
        self.detection_system = SineDetectionSystem(config_manager)
        
        # Initialize BLE beacon for status broadcasting
        self.ble_beacon = BLEAdvertisingBeacon(
            device_name="DRONE_STATUS",
            use_extended_advertising=False,
            message_in_device_name=False  # Use manufacturer data for status
        )
        
        # Status tracking
        self.last_broadcast_status = None
        self.broadcast_count = 0
        self.last_broadcast_time = 0
        self.broadcast_interval = 1.0  # Broadcast every 1 second
        
        print("Drone Status Broadcaster initialized")
        print("📡 BLE Device Name: DRONE_STATUS")
        print("📊 Broadcasting: START/STOP status via manufacturer data")
    
    def start(self):
        """Start the detection system and BLE beacon"""
        # Start detection system (includes IMU handler)
        if not self.detection_system.start():
            print("ERROR: Failed to start detection system")
            return False
        print("SUCCESS: Detection system started")
        
        # Start BLE beacon
        if not self.ble_beacon.start_advertising("INIT"):
            print("ERROR: Failed to start BLE beacon")
            return False
        print("SUCCESS: BLE beacon started")
        
        return True
    
    def stop(self):
        """Stop the detection system and BLE beacon"""
        self.detection_system.stop()
        self.ble_beacon.stop_advertising()
        print("STOP: Drone status broadcaster stopped")
    
    def broadcast_status(self, status):
        """Broadcast drone status via BLE"""
        current_time = utime.time()
        
        # Only broadcast if status changed or enough time passed
        if (status != self.last_broadcast_status or 
            current_time - self.last_broadcast_time >= self.broadcast_interval):
            
            # Create simple status message: "1" for START, "0" for STOP
            if status == "START":
                status_message = "1"
            else:  # STOP
                status_message = "0"
            
            # Update BLE beacon
            if self.ble_beacon.update_message(status_message):
                self.broadcast_count += 1
                self.last_broadcast_status = status
                self.last_broadcast_time = current_time
                
                print("📡 BROADCAST #{}: {} ({})".format(
                    self.broadcast_count, status_message, status
                ))
                return True
            else:
                print("❌ Failed to broadcast status: {}".format(status))
                return False
        
        return True  # No broadcast needed
    
    def run_broadcast_loop(self, max_duration_seconds=300, update_rate_ms=50):
        """Run the integrated detection and broadcasting loop"""
        print("Starting drone status broadcasting...")
        print("📡 Broadcasting status every {} seconds".format(self.broadcast_interval))
        
        start_time = utime.time()
        
        try:
            while True:
                current_time = utime.time()
                
                # Get IMU sample from detection system
                sample = self.detection_system.get_imu_sample()
                
                # Process sample and get state
                state = self.detection_system.detector.process_sample(sample)
                
                # Get current drone status
                current_status = self.detection_system.detector.get_drone_status()
                
                # Broadcast status
                self.broadcast_status(current_status)
                
                # Check BLE events (non-blocking)
                self.ble_beacon.check_events()
                
                # Debug output every 10 samples
                if self.detection_system.detector.sample_count % 10 == 0:
                    print("[{}] State: {} | Status: {} | AZ={:.3f} AX={:.3f} AY={:.3f}".format(
                        self.detection_system.detector.sample_count,
                        self.detection_system.detector.get_state_name(),
                        current_status,
                        sample['az'], sample['ax'], sample['ay']
                    ))
                
                # Check timeout (only if no takeoff detected yet)
                if current_status == "STOP" and current_time - start_time > max_duration_seconds:
                    print("TIMEOUT: No takeoff detected in {} seconds".format(max_duration_seconds))
                    break
                
                utime.sleep_ms(update_rate_ms)
                
        except KeyboardInterrupt:
            print("STOP: Broadcasting stopped by user")
        except Exception as e:
            print("ERROR: {}".format(e))
        finally:
            # Print final summary
            print("\n=== FINAL SUMMARY ===")
            print("Total samples processed: {}".format(self.detection_system.detector.sample_count))
            print("State changes: {}".format(self.detection_system.detector.state_change_count))
            print("Reset count: {}".format(self.detection_system.detector.reset_count))
            print("Final drone status: {}".format(self.detection_system.detector.get_drone_status()))
            print("Total broadcasts: {}".format(self.broadcast_count))
            print("Total runtime: {:.2f} seconds".format(utime.time() - start_time))
            self.stop()


# Main execution
if __name__ == "__main__":
    try:
        from usr.config_manager import ConfigManager
        config_mgr = ConfigManager()
        broadcaster = DroneStatusBroadcaster(config_mgr)
        
        if not broadcaster.start():
            print("ERROR: Failed to start drone status broadcaster")
            exit(1)
        
        print("Starting drone status broadcasting...")
        broadcaster.run_broadcast_loop(max_duration_seconds=300, update_rate_ms=50)  # 5 minutes monitoring
            
    except Exception as e:
        print("ERROR: {}".format(e)) 