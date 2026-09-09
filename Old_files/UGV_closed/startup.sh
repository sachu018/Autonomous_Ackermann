#!/bin/bash
# startup.sh
# ----------
# System startup script for the RTK-Only Closed-Loop UGV.
# Installs as a systemd startup task managed by robot.service.
#
# Responsibilities:
#   1. Configure pinmux for GPIO, I2C, and UART peripherals.
#   2. Export required GPIO pins to sysfs.
#   3. Clean up log directory and timestamp logs.
#   4. Launch the Python main.py script.

set -e

# Force Adafruit Blinka to override BBB board detection checks
export BLINKA_FORCEBOARD=BEAGLEBONE_BLACK

echo "[Startup] RTK Closed-Loop UGV controller starting..."
cd /home/debian/Robot/UGV_closed

# -- Pinmux Configuration --------------------------------------------------------
echo "[Startup] Configuring GPIO and peripheral pinmux..."
config-pin P8_7  gpio   # REV_LEFT (Left motor reverse)
config-pin P8_8  gpio   # BRAKE_LEFT (Left motor brake)
config-pin P8_9  gpio   # REV_RIGHT (Right motor reverse)
config-pin P8_10 gpio   # BRAKE_RIGHT (Right motor brake)
config-pin P8_14 gpio   # CONTACTOR (Main contactor relay)
config-pin P9_19 i2c    # I2C2_SCL (DAC and absolute orientation)
config-pin P9_20 i2c    # I2C2_SDA (DAC and absolute orientation)
config-pin P8_37 uart   # UART5_TX (FlySky iBUS Receiver)
config-pin P8_38 uart   # UART5_RX (FlySky iBUS Receiver)
echo "[Startup] Pinmux configuration complete."

# -- GPIO Sysfs Export -----------------------------------------------------------
# Pins:
#   gpio26  = P8_14  (CONTACTOR)
#   gpio66  = P8_7   (REV_LEFT)
#   gpio67  = P8_8   (BRAKE_LEFT)
#   gpio69  = P8_9   (REV_RIGHT)
#   gpio68  = P8_10  (BRAKE_RIGHT)
echo "[Startup] Exporting GPIO pins to sysfs..."
for pin in 26 66 67 68 69; do
    if [ ! -d /sys/class/gpio/gpio${pin} ]; then
        echo ${pin} > /sys/class/gpio/export
        echo "[Startup] Exported gpio${pin}"
    else
        echo "[Startup] gpio${pin} already exported – skipping"
    fi
done

# Wait for sysfs configurations to settle
sleep 1
echo "[Startup] Hardware system registers ready."

# -- Logging Directory Setup -----------------------------------------------------
LOG_DIR="/home/debian/Robot/UGV_closed/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/ugv_closed_loop_$(date +%Y%m%d_%H%M%S).log"
echo "[Startup] Redirecting console print outputs to: ${LOG_FILE}"

# Keep only the 10 most recent logs to prevent SD card storage overflow
ls -1t "${LOG_DIR}"/ugv_closed_loop_*.log 2>/dev/null | tail -n +11 | xargs -r rm --
echo "[Startup] Old logs pruned (keeping last 10)."

# -- Launch python script --------------------------------------------------------
echo "[Startup] Launching main.py..."
exec python3 /home/debian/Robot/UGV_closed/main.py >> "${LOG_FILE}" 2>&1
