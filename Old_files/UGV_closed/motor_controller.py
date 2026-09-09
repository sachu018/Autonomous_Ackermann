# motor_controller.py
# High-level motor drive sequencer that interfaces with the low-level BBBHardware module.

import time
from hardware import BBBHardware
from config import (
    DAC_LEFT_ADDR, DAC_RIGHT_ADDR, DAC_MAX_VALUE, DAC_ZERO_VALUE,
    GPIO_REV_LEFT, GPIO_REV_RIGHT, GPIO_BRAKE_LEFT, GPIO_BRAKE_RIGHT,
    GPIO_CONTACTOR, LEFT_MOTOR_DIR, RIGHT_MOTOR_DIR,
    MOTOR_SEQ_DELAY, REVERSE_BRAKE_DURATION, WHEEL_RPM_PHYS_MAX
)

class MotorController:
    def __init__(self):
        self.hw = BBBHardware()
        self.braking = False
        self.is_energized = False
        self.armed = False
        self._was_reversing = False
        self.last_dir_L = False
        self.last_dir_R = False
        
        self.last_dac_L = 0
        self.last_dac_R = 0
        
        # Safe startup initialization
        self.deenergize_system()

    def energize_system(self):
        """Closes the main contactor relay to supply battery power to BLDC drivers."""
        if self.is_energized:
            return
            
        print("[MotorController] Energizing system (closing contactor P8_14)...")
        self.hw.set_gpio(GPIO_CONTACTOR, True)
        time.sleep(0.5)  # Wait for contactor coil to close and stabilize voltage
        
        print("[MotorController] Releasing hardware brakes...")
        self.hw.set_gpio(GPIO_BRAKE_LEFT, False)
        self.hw.set_gpio(GPIO_BRAKE_RIGHT, False)
        time.sleep(0.1)
        
        self.is_energized = True
        self.armed = True
        print("[MotorController] System energized and armed.")

    def deenergize_system(self):
        """Opens the main contactor relay to isolate battery power for safety."""
        if not self.is_energized and self.armed == False:
            # Run at startup to make sure contactor is forced open
            self.hw.set_gpio(GPIO_CONTACTOR, False)
            return
            
        print("[MotorController] De-energizing system (opening contactor P8_14)...")
        self.stop()
        self.hw.set_gpio(GPIO_CONTACTOR, False)
        self.is_energized = False
        self.armed = False
        print("[MotorController] System de-energized (Safe State).")

    def set_motors(self, rpm_L: float, rpm_R: float):
        """
        Drives the motors at target RPMs. Handles direction swaps, deadband feedforward, and slew-rate limits.
        """
        # Apply direction multipliers
        rpm_L_mod = rpm_L * LEFT_MOTOR_DIR
        rpm_R_mod = rpm_R * RIGHT_MOTOR_DIR

        new_dir_L = rpm_L_mod < 0
        new_dir_R = rpm_R_mod < 0

        # Check if direction changes (triggers safety sequence to bypass lockouts)
        dir_changed = (new_dir_L != self.last_dir_L) or (new_dir_R != self.last_dir_R)

        if dir_changed:
            # 1. Zero out throttle first
            self.hw.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
            self.hw.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
            self.last_dac_L = 0
            self.last_dac_R = 0
            time.sleep(0.01)

            # 2. Toggle direction pins while at 0 speed
            self.hw.set_gpio(GPIO_REV_LEFT, new_dir_L)
            self.hw.set_gpio(GPIO_REV_RIGHT, new_dir_R)
            self.last_dir_L = new_dir_L
            self.last_dir_R = new_dir_R

            # 3. Ensure brakes are off
            self.hw.set_gpio(GPIO_BRAKE_LEFT, False)
            self.hw.set_gpio(GPIO_BRAKE_RIGHT, False)

            # 4. Wait for e-scooter driver internal controller logic to acknowledge direction change
            time.sleep(0.15)
        else:
            # No direction change - ensure correct directions and brakes released
            self.hw.set_gpio(GPIO_REV_LEFT, new_dir_L)
            self.hw.set_gpio(GPIO_REV_RIGHT, new_dir_R)
            self.hw.set_gpio(GPIO_BRAKE_LEFT, False)
            self.hw.set_gpio(GPIO_BRAKE_RIGHT, False)

        # Sequence delay to allow driver transitions to settle
        time.sleep(MOTOR_SEQ_DELAY)

        # 5. Convert target RPM to DAC commands
        target_dac_L = int((abs(rpm_L) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)
        target_dac_R = int((abs(rpm_R) / WHEEL_RPM_PHYS_MAX) * DAC_MAX_VALUE)

        # 6. Apply Feedforward deadband compensation (+1100 raw points for active demands)
        if abs(rpm_L) > 0.01:
            target_dac_L = max(1100, target_dac_L)
        else:
            target_dac_L = 0
            
        if abs(rpm_R) > 0.01:
            target_dac_R = max(1100, target_dac_R)
        else:
            target_dac_R = 0

        # Clamp DAC command limits
        target_dac_L = max(0, min(DAC_MAX_VALUE, target_dac_L))
        target_dac_R = max(0, min(DAC_MAX_VALUE, target_dac_R))

        # 7. Apply Slew Rate Limiting (max change of 450 points per cycle)
        max_step = 450
        
        diff_L = target_dac_L - self.last_dac_L
        if diff_L > max_step:
            dac_L = self.last_dac_L + max_step
        elif diff_L < -max_step:
            dac_L = self.last_dac_L - max_step
        else:
            dac_L = target_dac_L
            
        diff_R = target_dac_R - self.last_dac_R
        if diff_R > max_step:
            dac_R = self.last_dac_R + max_step
        elif diff_R < -max_step:
            dac_R = self.last_dac_R - max_step
        else:
            dac_R = target_dac_R

        # Update cached last writes
        self.last_dac_L = dac_L
        self.last_dac_R = dac_R

        # Write to DACs
        self.hw.write_dac(DAC_LEFT_ADDR, dac_L)
        self.hw.write_dac(DAC_RIGHT_ADDR, dac_R)
        self.braking = False

    def brake(self):
        """Instantly zero out throttle and pull hardware brakes HIGH."""
        self.hw.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        self.hw.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.hw.set_gpio(GPIO_BRAKE_LEFT, True)
        self.hw.set_gpio(GPIO_BRAKE_RIGHT, True)
        self.hw.set_gpio(GPIO_REV_LEFT, False)
        self.hw.set_gpio(GPIO_REV_RIGHT, False)
        self.braking = True

    def stop(self):
        """Zero out speed command but release brakes."""
        self.hw.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        self.hw.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.hw.set_gpio(GPIO_BRAKE_LEFT, False)
        self.hw.set_gpio(GPIO_BRAKE_RIGHT, False)
        self.hw.set_gpio(GPIO_REV_LEFT, False)
        self.hw.set_gpio(GPIO_REV_RIGHT, False)
        self.braking = False

    def brake_then_reverse(self, rpm_L: float, rpm_R: float):
        """Engages brakes for a controlled period before swapping directions and driving reverse."""
        self.hw.write_dac(DAC_LEFT_ADDR, DAC_ZERO_VALUE)
        self.hw.write_dac(DAC_RIGHT_ADDR, DAC_ZERO_VALUE)
        self.last_dac_L = 0
        self.last_dac_R = 0
        self.hw.set_gpio(GPIO_BRAKE_LEFT, True)
        self.hw.set_gpio(GPIO_BRAKE_RIGHT, True)
        self.hw.set_gpio(GPIO_REV_LEFT, False)
        self.hw.set_gpio(GPIO_REV_RIGHT, False)
        self.braking = True

        time.sleep(REVERSE_BRAKE_DURATION)

        self.hw.set_gpio(GPIO_BRAKE_LEFT, False)
        self.hw.set_gpio(GPIO_BRAKE_RIGHT, False)
        self.braking = False

        self.set_motors(rpm_L, rpm_R)

    def close(self):
        """Safe shutdown."""
        self.stop()
        self.deenergize_system()
        self.hw.close()
