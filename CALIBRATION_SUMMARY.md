# Drone Detection Calibration Summary

## 🎯 **Calibration for Small Drone with Low Movement**

### **📊 Threshold Changes (Z-Axis More Sensitive)**

| Parameter          | Original | New (Calibrated) |       Change      |
|--------------------|----------|------------------|-------------------|
| `IDLE_THRESH`      | 0.2g     | 0.05g            | **75% reduction** |
| `LARGE_THRESH`     | 2.0g     | 1.5g             | **25% reduction** |
| `MARGIN`           | 0.10g    | 0.05g            | **50% reduction** |
| `MIN_AMPLITUDE`    | 0.10g    | 0.05g            | **50% reduction** |
| Window filter      | 3.0g     | 2.0g             | **33% reduction** |

### **⏱️ NEW: Time-Based Transition Thresholds**

| Parameter            |     Value   |                 Purpose                    |
|----------------------|-------------|--------------------------------------------|
| `TRANSITION_TIMEOUT` | 5.0 seconds | Max time between specific sine wave states |

### **🔧 Specific Changes Made**

#### **1. Z-Axis Thresholds (More Sensitive)**
- **Idle detection:** 0.05g (was 0.09g) - **75% reduction**
- **Large movement:** 1.5g (was 0.9g) - **increased to prevent resets**
- **Trend detection:** 0.05g margin (was 0.10g)
- **Amplitude detection:** 0.05g minimum (was 0.10g)

#### **2. NEW: State-Dependent Thresholds**
- **Takeoff states:** 2.0g threshold (FIRST_FALL, SECOND_FALL, SECOND_RISE)
- **Other states:** 1.5g threshold (IDLE, MOTOR_ON, FIRST_RISE)
- **Prevents resets** during normal takeoff movements

#### **3. NEW: Sensitive Motor Detection**
- **Movement threshold:** 0.02g (very low for motor detection)
- **Gyro threshold:** 5.0 dps (detects motor vibrations)
- **Detection method:** Any axis movement OR gyro movement
- **Logging:** Detailed motor start detection messages

#### **4. IMPROVED: State Transition Logic**
- **STEADY → IDLE:** Requires 10.0 seconds of idle condition (was 5.0s)
- **Landing check:** 10-second monitoring period before confirming landing
- **Status management:** Immediate "STOP" when transitioning to IDLE
- **Motor stop detection:** Only in early states (MOTOR_ON, FIRST_RISE)
- **Motor stop threshold:** 0.01g (was 0.02g) - very strict for early states only

#### **5. Landing Detection**
- **STEADY_IDLE_THRESH:** 0.03g (more sensitive for landing)
- **STEADY_GYRO_THRESH:** 10.0 dps (lower for landing)
- **Separate method:** `in_steady_idle_condition()` for landing detection

#### **6. Time-Based Transitions**
- **FIRST_RISE → FIRST_FALL:** Max 5.0 seconds
- **FIRST_FALL → SECOND_FALL:** Max 5.0 seconds
- **SECOND_FALL → SECOND_RISE:** Max 5.0 seconds
- **Timeout behavior:** Reset to IDLE state
- **No timeout for:** IDLE→MOTOR_ON, MOTOR_ON→FIRST_RISE, SECOND_RISE→STEADY

#### **7. X/Y-Axis Thresholds (Unchanged)**
- **Idle detection:** 0.09g (same as Z-axis)
- **Large movement:** 0.9g (same as Z-axis)
- **Rotation thresholds:** 300 dps (kept same)

#### **8. Filtering Improvements**
- **Z-axis window filter:** 2.0g max (was 3.0g)
- **Allows smaller movements** to be processed

### **🎯 Expected Benefits**

#### **✅ For Small Drones:**
- **2x more sensitive** to Z-axis movements
- **Faster takeoff detection** with subtle movements
- **Better capture** of low-amplitude sine wave patterns
- **Reduced false negatives** for small drones
- **Quick timeout detection** prevents hanging in sine wave states

#### **⏱️ Time-Based Improvements:**
- **Prevents hanging** in critical sine wave transitions
- **Ensures quick progression** through sine wave pattern
- **Automatic reset** if sine wave pattern is incomplete
- **Better robustness** against partial sine wave movements

#### **⚠️ Considerations:**
- **May be more sensitive to noise:** Monitor for false positives
- **Test in real conditions:** Verify detection accuracy
- **Adjust timeouts if needed:** Based on your drone's takeoff speed
- **Monitor timeout resets:** Ensure they're not too frequent

### **🧪 Testing Recommendations**

1. **Test with your small drone** in real takeoff conditions
2. **Monitor debug output** for threshold triggers and timeouts
3. **Check for timeout resets** - should be minimal during proper takeoff
4. **Verify detection speed** is acceptable
5. **Adjust timeouts if needed** based on results

### **📈 Expected Detection Behavior**

```
Before Calibration:
- Required Z-axis movement: 0.2g+ for idle detection
- Required amplitude: 0.10g+ for trend detection
- Large movement reset: 2.0g+
- No time limits on state transitions

After Calibration:
- Required Z-axis movement: 0.09g+ for idle detection
- Required amplitude: 0.05g+ for trend detection  
- Large movement reset: 0.9g+
- 0.4s timeout for sine wave transitions only
- No timeout for other state transitions
```

### **🔄 Revert if Needed**

If the calibration is too sensitive, you can revert to original values:
- `IDLE_THRESH = 0.2`
- `LARGE_THRESH = 2.0`
- `MARGIN = 0.10`
- `MIN_AMPLITUDE = 0.10`
- Window filter: `abs(value) > 3.0`
- Remove time-based transitions if needed 