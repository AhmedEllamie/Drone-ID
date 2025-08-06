#!/usr/bin/env python3
"""
Improved Sleep Test for EC200A
- Addresses 60mA power consumption issue
- Fixes button debouncing
- Tries multiple power saving methods
"""

import pm
import utime
from machine import Pin, ExtInt

# GPIO18 is connected to WAKEUP_IN pin
WAKEUP_GPIO = 18
GPS_PWR_GPIO = 47

# Global variables
wakeup_triggered = False
last_interrupt_time = 0
debounce_time = 500  # 500ms debounce

def wakeup_callback(args):
    """Debounced callback function for wake-up interrupt"""
    global wakeup_triggered, last_interrupt_time
    
    current_time = utime.ticks_ms()
    
    # Debouncing: ignore if interrupt happened less than 500ms ago
    if utime.ticks_diff(current_time, last_interrupt_time) > debounce_time:
        print("\n🔔 WAKE-UP TRIGGERED! (debounced)")
        wakeup_triggered = True
        last_interrupt_time = current_time

def check_power_status():
    """Check what might be preventing low power consumption"""
    print("\n🔍 Checking power status...")
    
    # Check wakelocks
    wakelock_count = pm.get_wakelock_num()
    print("  - Wakelocks active: {}".format(wakelock_count))
    
    # Check autosleep status
    try:
        autosleep_status = pm.autosleep(0)  # Query current status
        print("  - Autosleep status: {}".format(autosleep_status))
    except:
        print("  - Cannot query autosleep status")
    
    return wakelock_count == 0

def force_power_saving():
    """Try multiple methods to reduce power consumption"""
    print("\n⚡ Attempting to reduce power consumption...")
    
    try:
        # Method 1: Ensure no wakelocks
        wakelock_count = pm.get_wakelock_num()
        if wakelock_count > 0:
            print("  ⚠ {} wakelocks detected - this prevents deep sleep".format(wakelock_count))
        
        # Method 2: Enable autosleep
        result = pm.autosleep(1)
        if result == 0:
            print("  ✓ Autosleep enabled")
        else:
            print("  ✗ Autosleep failed: {}".format(result))
    
        
        return True
        
    except Exception as e:
        print("  ✗ Power saving setup failed: {}".format(e))
        return False

def setup_gpio_wakeup():
    """Configure GPIO with proper debouncing"""
    global wakeup_pin, wakeup_extint
    
    print("Setting up GPIO{} wake-up interrupt...".format(WAKEUP_GPIO))
    
    try:
        # Configure GPIO as input with pull-up
        wakeup_pin = Pin(WAKEUP_GPIO, Pin.IN, Pin.PULL_PU, 0)
        print("✓ GPIO{} configured as input with pull-up".format(WAKEUP_GPIO))
        print("  Current GPIO{} level: {}".format(WAKEUP_GPIO, wakeup_pin.read()))
        
        # Set up external interrupt with debouncing
        wakeup_extint = ExtInt(WAKEUP_GPIO, ExtInt.IRQ_FALLING, ExtInt.PULL_PU, wakeup_callback)
        wakeup_extint.enable()
        print("✓ GPIO{} interrupt configured (falling edge, debounced)".format(WAKEUP_GPIO))
        
        return True
        
    except Exception as e:
        print("✗ Failed to setup GPIO{}: {}".format(WAKEUP_GPIO, e))
        return False

def enter_power_save_mode():
    """Enter power saving mode with monitoring"""
    global wakeup_triggered
    
    print("\n💤 ENTERING POWER SAVE MODE...")
    print("   Expected: 1-10mA consumption")
    print("   If still 60mA, check:")
    print("   - Active network connections")
    print("   - USB connection power")
    print("   - Other active peripherals")
    print("   Pull GPIO{} to GND to wake up!".format(WAKEUP_GPIO))
    
    wakeup_triggered = False
    sleep_duration = 0
    status_interval = 30  # Show status every 30 seconds
    
    while not wakeup_triggered and sleep_duration < 3600:  # Max 1 hour
        utime.sleep(1)
        sleep_duration += 1
        
        # Show status periodically
        if sleep_duration % status_interval == 0:
            print("   💤 Sleeping... {} minutes ({} sec)".format(
                sleep_duration // 60, sleep_duration))
    
    if wakeup_triggered:
        print("✅ Wake-up successful after {} seconds!".format(sleep_duration))
    else:
        print("⏰ Sleep timeout after {} seconds".format(sleep_duration))

def main():
    """Main test function with power debugging"""
    global wakeup_triggered
    gpspwr = Pin(GPS_PWR_GPIO, Pin.OUT, Pin.PULL_PU, 0)
    gpspwr.write(1)

    # Check power status
    if not check_power_status():
        print("\n⚠ Power status check failed")
    
    # Force power saving
    if not force_power_saving():
        print("\n❌ Power saving setup failed")
        return
    
    # Setup GPIO wake-up
    if not setup_gpio_wakeup():
        print("\n❌ GPIO setup failed")
        return
    
    print("\n✅ Setup complete - monitor power consumption now!")
    print("📏 Check power meter: should drop from ~200mA to ~1-10mA")
    
    cycle_count = 0
    
    try:
        while True:
            cycle_count += 1
            print("\n--- Power Save Cycle #{} ---".format(cycle_count))
            
            # Reset wake-up flag
            wakeup_triggered = False
            
            # Short countdown
            print("🔆 Active mode (should show ~60-200mA)")
            print("Entering power save in 10 seconds...")
            for i in range(10, 0, -1):
                if wakeup_triggered:
                    print("⚡ Wake-up during countdown!")
                    break
                print("  {}...".format(i))
                utime.sleep(1)
            
            # Enter power save mode
            if not wakeup_triggered:
                enter_power_save_mode()
            
            # Active period
            print("\n🔆 Active mode resumed")
            print("Power should return to ~60-200mA")
            print("Next cycle in 5 seconds...")
            
            for i in range(5, 0, -1):
                if wakeup_triggered:
                    wakeup_triggered = False  # Reset
                print("  {}...".format(i))
                utime.sleep(1)
                
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted")
    except Exception as e:
        print("\n❌ Error: {}".format(e))
    finally:
        try:
            wakeup_extint.disable()
            print("✓ GPIO interrupt disabled")
        except:
            pass
        print("✅ Test completed")

if __name__ == "__main__":
    main() 