"""
Test Imports - Verify all dependencies work correctly
"""

def test_imports():
    """Test all required imports"""
    print("Testing imports...")
    
    try:
        import utime
        print("✅ utime imported successfully")
    except ImportError as e:
        print("❌ utime import failed: {}".format(e))
        return False
    
    try:
        from new_algorithm_final import SineDetectionSystem
        print("✅ SineDetectionSystem imported successfully")
    except ImportError as e:
        print("❌ SineDetectionSystem import failed: {}".format(e))
        return False
    
    try:
        from anna_advertising_beacon import BLEAdvertisingBeacon
        print("✅ BLEAdvertisingBeacon imported successfully")
    except ImportError as e:
        print("❌ BLEAdvertisingBeacon import failed: {}".format(e))
        return False
    
    try:
        from usr.config_manager import ConfigManager
        print("✅ ConfigManager imported successfully")
    except ImportError as e:
        print("❌ ConfigManager import failed: {}".format(e))
        return False
    
    try:
        from drone_status_broadcaster import DroneStatusBroadcaster
        print("✅ DroneStatusBroadcaster imported successfully")
    except ImportError as e:
        print("❌ DroneStatusBroadcaster import failed: {}".format(e))
        return False
    
    print("\n🎉 All imports successful!")
    return True


if __name__ == "__main__":
    test_imports() 