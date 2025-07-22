"""
Demo Script – IMU Raw Readings Logger
-------------------------------------
This demo shows how to initialise the IMUHandler and continuously log raw
accelerometer, gyroscope, and orientation data.  It assumes the DTS project
is running on a QuecPython-compatible device with the required hardware
connected.

Usage:
    $ python imu_raw_demo.py         # (on device)

The script will run indefinitely until interrupted (Ctrl-C) and will print one
line per second containing the latest sensor readings.
"""

import utime
from usr.config_manager import ConfigManager
from usr.imu_handler import IMUHandler
from usr.modules.logging import getLogger


def main():
    """Initialise IMUHandler and periodically print raw readings."""
    log = getLogger("IMU-DEMO")

    # Load configuration (fallback to project root config.json if /usr not present)
    try:
        cfg_mgr = ConfigManager()  # default path: /usr/config.json
    except Exception:
        cfg_mgr = ConfigManager("config.json")
        log.warning("/usr/config.json not found – using local config.json instead")

    # Start IMU monitoring
    imu = IMUHandler(cfg_mgr)
    if not imu.start():
        log.error("Failed to start IMU handler – exiting demo")
        return

    log.info("IMU handler started – logging raw readings every second (Ctrl+C to stop)")

    # Detection parameters
    BASELINE_G = 1.0                              # g at rest
    TAKEOFF_ACCEL_THRESHOLD = 0.30                # g above baseline
    LANDING_IMPACT_THRESHOLD = 0.60               # g above baseline on impact
    GROUND_TOLERANCE = 0.12                       # |z-1g| within tolerance considered at rest
    POST_LANDING_STABLE_SEC = 2                   # stable duration to confirm landing
    SAMPLE_PERIOD_MS = 100                        # 100 ms sampling period

    flight_state = "ground"  # or "air"
    landing_candidate_ts = None
    last_log_ts = utime.ticks_ms()

    try:
        while True:
            accel = imu.get_accel()
            gyro = imu.get_gyro()
            orient = imu.get_orientation()

            az = accel["z"]  # current Z-axis acceleration (g)
            now_ms = utime.ticks_ms()

            # -------- TAKE-OFF / LANDING DETECTION --------
            if flight_state == "ground":
                # Detect sustained upward acceleration
                if az - BASELINE_G > TAKEOFF_ACCEL_THRESHOLD:
                    flight_state = "air"
                    log.info("*** TAKE-OFF detected (az={:+.3f} g) ***".format(az))
            else:  # in air
                # Monitor for landing impact (spike)
                if az - BASELINE_G > LANDING_IMPACT_THRESHOLD:
                    landing_candidate_ts = now_ms  # start candidate window
                # Confirm landing when acceleration stabilises near 1 g
                if landing_candidate_ts is not None:
                    if abs(az - BASELINE_G) <= GROUND_TOLERANCE:
                        if utime.ticks_diff(now_ms, landing_candidate_ts) >= POST_LANDING_STABLE_SEC * 1000:
                            flight_state = "ground"
                            landing_candidate_ts = None
                            log.info("*** LANDING detected (az={:+.3f} g) ***".format(az))
                    else:
                        # still fluctuating – reset timer
                        landing_candidate_ts = None

            # -------- PERIODIC LOGGING --------
            if utime.ticks_diff(now_ms, last_log_ts) >= 100:
                last_log_ts = now_ms
                log.info(
                    "ACCEL[g] x={:+.3f} y={:+.3f} z={:+.3f} | "
                    "GYRO[dps] x={:+.2f} y={:+.2f} z={:+.2f} | "
                    "ROLL={:+.2f}° PITCH={:+.2f}° HEAD={:+.2f}° | STATE={}".format(
                        accel["x"], accel["y"], accel["z"],
                        gyro["x"], gyro["y"], gyro["z"],
                        orient["roll"], orient["pitch"], orient["heading"],
                        flight_state.upper()
                    )
                )

            utime.sleep_ms(SAMPLE_PERIOD_MS)
    except KeyboardInterrupt:
        log.info("Keyboard interrupt received – stopping demo")
    finally:
        imu.stop()
        log.info("IMU demo stopped")


if __name__ == "__main__":
    main() 