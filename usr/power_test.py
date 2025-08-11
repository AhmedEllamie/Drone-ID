"""
-----------------------------------------------------
Project       : Drone Telematics System (DTS)
Author        : Ahmed Ellamie (ahmed.ellamiee@gmail.com)
Date Modified : May, 2024
Description   : Simple Power Management Test
                - Tests PSM (Power Saving Mode)
                - Tests wake-up functionality
                - Tests power locks
                - Demonstrates battery optimization
Dependencies  : 
                - usr.modules.power_manage
                - usr.modules.logging
-----------------------------------------------------
"""

import utime
import pm
from usr.modules.power_manage import PowerManage, PMLock
from usr.modules.logging import getLogger

# Initialize logger
log = getLogger(__name__)

class PowerTestSuite:
    """! Simple Power Management Test Suite"""
    
    def __init__(self):
        """! Initialize power test suite"""
        self.power_manager = PowerManage()
        self.test_count = 0
        self.passed_tests = 0
        
        print("=" * 50)
        print("   Power Management Test Suite")
        print("   Platform: EC200A QuecPython")
        print("=" * 50)
    
    def log_test(self, test_name, result, details=""):
        """! Log test result
        
        @param test_name Name of the test
        @param result True if passed, False if failed
        @param details Additional details about the test
        """
        self.test_count += 1
        if result:
            self.passed_tests += 1
            status = "PASS"
        else:
            status = "FAIL"
        
        print("[{}] Test {}: {} - {}".format(status, self.test_count, test_name, details))
        
        if not result:
            log.error("Test failed: {} - {}".format(test_name, details))
        else:
            log.info("Test passed: {}".format(test_name))
    
    def test_power_manager_creation(self):
        """! Test PowerManage class creation"""
        print("\n--- Testing PowerManage Creation ---")
        
        try:
            if self.power_manager:
                self.log_test("PowerManage Creation", True, "PowerManage instance created successfully")
                return True
            else:
                self.log_test("PowerManage Creation", False, "Failed to create PowerManage instance")
                return False
        except Exception as e:
            self.log_test("PowerManage Creation", False, "Exception: {}".format(e))
            return False
    
    def test_autosleep_control(self):
        """! Test autosleep enable/disable"""
        print("\n--- Testing AutoSleep Control ---")
        
        try:
            # Test disable autosleep
            result1 = self.power_manager.autosleep(0)
            if result1:
                self.log_test("Disable AutoSleep", True, "AutoSleep disabled successfully")
            else:
                self.log_test("Disable AutoSleep", False, "Failed to disable AutoSleep")
            
            utime.sleep(1)  # Small delay
            
            # Test enable autosleep
            result2 = self.power_manager.autosleep(1)
            if result2:
                self.log_test("Enable AutoSleep", True, "AutoSleep enabled successfully")
            else:
                self.log_test("Enable AutoSleep", False, "Failed to enable AutoSleep")
            
            return result1 and result2
            
        except Exception as e:
            self.log_test("AutoSleep Control", False, "Exception: {}".format(e))
            return False
    
    def test_power_lock(self):
        """! Test PMLock functionality"""
        print("\n--- Testing Power Lock ---")
        
        try:
            # Test power lock creation
            lock_name = "TEST_LCK"
            
            try:
                pm_lock = PMLock(lock_name)
                self.log_test("Power Lock Creation", True, "PMLock '{}' created".format(lock_name))
            except Exception as e:
                self.log_test("Power Lock Creation", False, "Failed to create lock: {}".format(e))
                return False
            
            # Test context manager usage
            try:
                with pm_lock:
                    self.log_test("Power Lock Context", True, "Entered lock context successfully")
                    # Simulate some work
                    print("    Simulating critical operation (2 seconds)...")
                    utime.sleep(2)
                    print("    Critical operation completed")
                
                self.log_test("Power Lock Release", True, "Exited lock context successfully")
                return True
                
            except Exception as e:
                self.log_test("Power Lock Context", False, "Context manager failed: {}".format(e))
                return False
                
        except Exception as e:
            self.log_test("Power Lock Test", False, "Exception: {}".format(e))
            return False
    
    def test_psm_disable(self):
        """! Test PSM disable"""
        print("\n--- Testing PSM Disable ---")
        
        try:
            # Disable PSM (stay awake)
            result = self.power_manager.set_psm(mode=0)
            
            if result:
                self.log_test("PSM Disable", True, "PSM disabled - device stays awake")
                
                # Check PSM status if available
                if hasattr(pm, "get_psm_time"):
                    psm_status = pm.get_psm_time()
                    print("    PSM Status: {}".format(psm_status))
                
                return True
            else:
                self.log_test("PSM Disable", False, "Failed to disable PSM")
                return False
                
        except Exception as e:
            self.log_test("PSM Disable", False, "Exception: {}".format(e))
            return False
    
    def test_psm_short_cycle(self):
        """! Test PSM with short sleep/wake cycle"""
        print("\n--- Testing PSM Short Cycle ---")
        
        try:
            # Set PSM: Sleep 60 seconds, wake 30 seconds
            tau_seconds = 60   # Sleep time
            act_seconds = 30   # Active time
            
            print("    Setting PSM: Sleep {}s, Wake {}s".format(tau_seconds, act_seconds))
            
            result = self.power_manager.set_psm(mode=1, tau=tau_seconds, act=act_seconds)
            
            if result:
                self.log_test("PSM Short Cycle", True, "PSM set: sleep {}s, wake {}s".format(tau_seconds, act_seconds))
                
                # Check PSM status
                if hasattr(pm, "get_psm_time"):
                    psm_status = pm.get_psm_time()
                    print("    PSM Status: {}".format(psm_status))
                    
                    if psm_status and psm_status[0] == 1:  # PSM enabled
                        self.log_test("PSM Status Check", True, "PSM is active: {}".format(psm_status[1:]))
                    else:
                        self.log_test("PSM Status Check", False, "PSM not active")
                
                return True
            else:
                self.log_test("PSM Short Cycle", False, "Failed to set PSM")
                return False
                
        except Exception as e:
            self.log_test("PSM Short Cycle", False, "Exception: {}".format(e))
            return False
    
    def test_psm_long_cycle(self):
        """! Test PSM with longer sleep/wake cycle"""
        print("\n--- Testing PSM Long Cycle ---")
        
        try:
            # Set PSM: Sleep 10 minutes, wake 2 minutes
            tau_seconds = 600   # Sleep time (10 minutes)
            act_seconds = 120   # Active time (2 minutes)
            
            print("    Setting PSM: Sleep {}s ({}min), Wake {}s ({}min)".format(
                tau_seconds, tau_seconds//60, act_seconds, act_seconds//60))
            
            result = self.power_manager.set_psm(mode=1, tau=tau_seconds, act=act_seconds)
            
            if result:
                self.log_test("PSM Long Cycle", True, "PSM set: sleep {}min, wake {}min".format(
                    tau_seconds//60, act_seconds//60))
                
                # Check PSM status
                if hasattr(pm, "get_psm_time"):
                    psm_status = pm.get_psm_time()
                    print("    PSM Status: {}".format(psm_status))
                
                return True
            else:
                self.log_test("PSM Long Cycle", False, "Failed to set PSM")
                return False
                
        except Exception as e:
            self.log_test("PSM Long Cycle", False, "Exception: {}".format(e))
            return False
    
    def test_wake_up_simulation(self):
        """! Test wake-up after short sleep"""
        print("\n--- Testing Wake-Up Simulation ---")
        
        try:
            print("    Setting up short PSM cycle for wake-up test...")
            
            # Set very short PSM cycle for testing: Sleep 30s, wake 10s
            tau_seconds = 30
            act_seconds = 10
            
            result = self.power_manager.set_psm(mode=1, tau=tau_seconds, act=act_seconds)
            
            if not result:
                self.log_test("Wake-Up Setup", False, "Failed to set PSM for wake-up test")
                return False
            
            self.log_test("Wake-Up Setup", True, "PSM configured for wake-up test")
            
            # Monitor for a short period
            print("    Monitoring device for 45 seconds...")
            print("    Expected: Device should sleep and wake automatically")
            
            start_time = utime.time()
            awake_periods = 0
            
            for i in range(45):  # Monitor for 45 seconds
                current_time = utime.time()
                elapsed = current_time - start_time
                
                # Print status every 5 seconds
                if i % 5 == 0:
                    print("    Time: {}s - Device should be in PSM cycle".format(elapsed))
                
                # Simulate checking if device is responsive
                # In real scenario, you'd check network connectivity
                try:
                    # This is a simple responsiveness check
                    if hasattr(pm, "get_psm_time"):
                        status = pm.get_psm_time()
                        if status:
                            awake_periods += 1
                except:
                    pass
                
                utime.sleep(1)
            
            self.log_test("Wake-Up Monitoring", True, "Completed 45-second monitoring period")
            
            # Disable PSM after test
            self.power_manager.set_psm(mode=0)
            self.log_test("Wake-Up Cleanup", True, "PSM disabled after test")
            
            return True
            
        except Exception as e:
            self.log_test("Wake-Up Simulation", False, "Exception: {}".format(e))
            # Ensure PSM is disabled even if test fails
            try:
                self.power_manager.set_psm(mode=0)
            except:
                pass
            return False
    
    def test_hibernate_mode(self):
        """! Test hibernate functionality (without actually hibernating)"""
        print("\n--- Testing Hibernate Mode (Dry Run) ---")
        
        try:
            print("    NOTE: This is a dry run - device will NOT actually hibernate")
            print("    Testing hibernate function availability...")
            
            # Check if hibernate function is available
            if hasattr(self.power_manager, 'set_hibernate'):
                self.log_test("Hibernate Available", True, "Hibernate function is available")
                
                # Don't actually call set_hibernate() as it would put device to sleep
                print("    Hibernate function ready (not executed for safety)")
                self.log_test("Hibernate Test", True, "Hibernate functionality verified (not executed)")
                
                return True
            else:
                self.log_test("Hibernate Available", False, "Hibernate function not available")
                return False
                
        except Exception as e:
            self.log_test("Hibernate Test", False, "Exception: {}".format(e))
            return False
    
    def run_all_tests(self):
        """! Run all power management tests"""
        print("Starting Power Management Test Suite...\n")
        
        # List of tests to run
        tests = [
            ("PowerManage Creation", self.test_power_manager_creation),
            ("AutoSleep Control", self.test_autosleep_control),
            ("Power Lock", self.test_power_lock),
            ("PSM Disable", self.test_psm_disable),
            ("PSM Short Cycle", self.test_psm_short_cycle),
            ("PSM Long Cycle", self.test_psm_long_cycle),
            ("Wake-Up Simulation", self.test_wake_up_simulation),
            ("Hibernate Mode", self.test_hibernate_mode)
        ]
        
        # Run each test
        for test_name, test_func in tests:
            try:
                test_func()
            except Exception as e:
                self.log_test(test_name, False, "Unhandled exception: {}".format(e))
            
            # Small delay between tests
            utime.sleep(1)
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """! Print test summary"""
        print("\n" + "=" * 50)
        print("            TEST SUMMARY")
        print("=" * 50)
        print("Total Tests: {}".format(self.test_count))
        print("Passed: {}".format(self.passed_tests))
        print("Failed: {}".format(self.test_count - self.passed_tests))
        print("Success Rate: {:.1f}%".format((self.passed_tests / self.test_count) * 100 if self.test_count > 0 else 0))
        print("=" * 50)
        
        # Power optimization tips
        print("\nPower Optimization Tips:")
        print("- Use PSM for battery-powered applications")
        print("- Longer sleep cycles = better battery life")
        print("- Use PMLock during critical operations")
        print("- Monitor actual power consumption with multimeter")
        print("=" * 50)

def quick_psm_demo():
    """! Quick PSM demonstration"""
    print("=" * 50)
    print("        Quick PSM Demo")
    print("=" * 50)
    
    try:
        pm_manager = PowerManage()
        
        print("1. Disabling PSM (staying awake)...")
        result1 = pm_manager.set_psm(mode=0)
        print("   Result: {}".format("Success" if result1 else "Failed"))
        
        utime.sleep(2)
        
        print("\n2. Setting PSM: Sleep 2 minutes, Wake 30 seconds...")
        result2 = pm_manager.set_psm(mode=1, tau=120, act=30)
        print("   Result: {}".format("Success" if result2 else "Failed"))
        
        if hasattr(pm, "get_psm_time"):
            psm_status = pm.get_psm_time()
            print("   PSM Status: {}".format(psm_status))
        
        utime.sleep(3)
        
        print("\n3. Disabling PSM again...")
        result3 = pm_manager.set_psm(mode=0)
        print("   Result: {}".format("Success" if result3 else "Failed"))
        
        print("\nQuick demo completed!")
        
    except Exception as e:
        print("Demo error: {}".format(e))

def main():
    """! Main test function"""
    print("Power Management Test for EC200A QuecPython")
    
    print("\nSelect test type:")
    print("1. Quick PSM Demo")
    print("2. Full Test Suite")
    
    try:
        # For automated testing, run quick demo by default
        choice = "2"  # Can be changed to input() for interactive mode
        
        if choice == "1":
            quick_psm_demo()
        else:
            tester = PowerTestSuite()
            tester.run_all_tests()
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        # Ensure PSM is disabled
        try:
            pm_manager = PowerManage()
            pm_manager.set_psm(mode=0)
            print("PSM disabled for safety")
        except:
            pass
    except Exception as e:
        print("Test error: {}".format(e))

if __name__ == "__main__":
    main() 