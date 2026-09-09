# config.py
# Central configuration file for the RTK closed-loop UGV control system.

import os

# ─────────────────────────────────────────────
#  System Paths
# ─────────────────────────────────────────────
BASE_DIR = "/home/debian/Robot/UGV_closed" if os.path.exists("/home/debian/Robot/UGV_closed") else os.path.dirname(os.path.abspath(__file__))
WAYPOINT_CSV = os.path.join(BASE_DIR, "waypoints_1m.csv")
LOG_DIR = os.path.join(BASE_DIR, "logs")

# ─────────────────────────────────────────────
#  Robot Geometry & Kinematic Limits
# ─────────────────────────────────────────────
R = 0.175          # Wheel radius (meters)
L = 0.6            # Track width / distance between wheels (meters)
V_MAX = 0.366      # Maximum allowed linear velocity (m/s)
W_MAX = 1.22       # Maximum allowed angular velocity (rad/s)

RPM_MAX = 400.0    # Maximum motor shaft RPM
GEAR_RATIO = 20.0  # Gearbox reduction ratio (1:20)
WHEEL_RPM_PHYS_MAX = RPM_MAX / GEAR_RATIO  # Physical maximum wheel RPM (20.0 RPM)
WHEEL_RPM_MAX = 10.0  # Command cap for wheel RPM (safety speed limit)
 
# RTK GPS Antenna Offsets (relative to the UGV's center of rotation)
# RTK_OFFSET_X: positive forward from center of rotation (in meters)
# RTK_OFFSET_Y: positive to the right of center of rotation (in meters)
RTK_OFFSET_X = 0.37
RTK_OFFSET_Y = -0.01

MIN_RATIO = 0.2    # Minimum speed ratio between wheels (prevents lockup during sharp differential turns)
MOTOR_SEQ_DELAY = 0.02  # Sequence delay in seconds when command is sent (20 ms)
REVERSE_BRAKE_DURATION = 0.5  # Seconds to wait at brake before engaging reverse direction

# ─────────────────────────────────────────────
#  Guidance & Waypoint Controller Tuning
# ─────────────────────────────────────────────
KP_LIN = 0.8       # Proportional gain for linear velocity
KP_ANG = 0.50      # Proportional gain for angular velocity (tuned for < 2.5cm mean CTE)
WP_THRESH = 0.4    # Distance threshold to switch to next waypoint (meters)
YAW_BOUND_DEG = 25.0  # Bounded yaw error threshold in degrees

YAW_METHOD = "LOOK_AHEAD"      # Waypoint target mode: "LOOK_AHEAD" (vector pursuit) or "DIRECT_WAYPOINT"
LOOK_AHEAD_DIST = 0.8          # Look-ahead distance for 1m sliced waypoints (0.8m prevents segment overshoots & clamps)
MIN_MOVE_DIST = 0.01           # Distance threshold in meters to compute/update yaw (reduced to 1cm for continuous updates)
COORDINATE_SMOOTHING_ALPHA = 0.4  # Coordinate smoothing filter factor (higher = less delay, lower = smoother)

# ─────────────────────────────────────────────
#  RTK Server Configuration (RPi Connection)
# ─────────────────────────────────────────────
RTK_HOST = "192.168.50.1"      # Raspberry Pi IP address
RTK_PORT = 6000                # TCP port for streaming NMEA/RTK coordinates
RTK_TIMEOUT_S = 1.0            # Network data loss timeout (seconds)

# ─────────────────────────────────────────────
#  GPIO Pin Assignments (BeagleBone Black)
# ─────────────────────────────────────────────
GPIO_CONTACTOR = "P8_14"   # GPIO 26 - Main power contactor relay
GPIO_REV_LEFT = "P8_7"     # GPIO 66 - Left motor reverse pin
GPIO_REV_RIGHT = "P8_9"    # GPIO 69 - Right motor reverse pin
GPIO_BRAKE_LEFT = "P8_8"   # GPIO 67 - Left motor active-low brake pin
GPIO_BRAKE_RIGHT = "P8_10" # GPIO 68 - Right motor active-low brake pin

# ─────────────────────────────────────────────
#  FlySky FS-i6X Transmitter iBUS UART Settings
# ─────────────────────────────────────────────
IBUS_PORT = "/dev/ttyS5"       # UART5 serial port device node
IBUS_BAUD = 115200             # Baudrate
IBUS_CH_X = 0                  # Channel 1: Steering (Left/Right)
IBUS_CH_Y = 1                  # Channel 2: Throttle (Forward/Reverse)
IBUS_CH_SWC = 4                # Channel 5: Mode selection (Manual vs Auto)
IBUS_CH_SWD = 5                # Channel 6: System arming switch (Contactor)
IBUS_CH_SWB = 6                # Channel 7: Reverse speed limit switch

RC_TIMEOUT_S = 0.4             # RC signal loss timeout (seconds)
RC_STICK_OVERRIDE_THRESH = 0.15  # Stick deflection threshold (15%) to trigger manual override

# ─────────────────────────────────────────────
#  MCP4725 I2C DAC Addresses & Values
# ─────────────────────────────────────────────
DAC_LEFT_ADDR = 0x61
DAC_RIGHT_ADDR = 0x60
DAC_MAX_VALUE = 4095           # 12-bit max
DAC_ZERO_VALUE = 0

# ─────────────────────────────────────────────
#  Motor Direction Multipliers
# ─────────────────────────────────────────────
LEFT_MOTOR_DIR = 1
RIGHT_MOTOR_DIR = 1

# ─────────────────────────────────────────────
#  FlySky Receiver Configuration Constants
# ─────────────────────────────────────────────
DEADBAND = 0.05
SWB_DEFAULT_SPEED = 1.0

