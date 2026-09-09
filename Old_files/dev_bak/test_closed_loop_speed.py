import time
import signal
import sys

# 1. Import the new closed-loop motor driver and encoder monitor
from drivers.motor_controller_closed_loop import ClosedLoopMotorController
from encoder_monitor import BBBEncoder

# Hardware parameters
CPR = 2400
LEFT_MOTOR_DIR = 1
RIGHT_MOTOR_DIR = 1

def main():
    print("Initializing Closed-Loop Speed Control Test...")
    
    # 2. Instantiate the driver and encoders
    motors = ClosedLoopMotorController()
    left_enc = BBBEncoder(1)  # eQEP1
    right_enc = BBBEncoder(2) # eQEP2
    
    # Setup exit signal handler
    def sig_handler(sig, frame):
        print("\nStopping motors and exiting...")
        motors.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, sig_handler)
    
    # 3. Energize contactor and release brakes
    print("Energizing contactor...")
    motors.energize_system()
    
    # Target speeds (commanding 5.0 RPM on both wheels)
    cmd_rpm_L = 5.0
    cmd_rpm_R = 5.0
    
    print(f"Running motors at {cmd_rpm_L} RPM for 5 seconds...")
    
    last_counts = [left_enc.read(), right_enc.read()]
    last_time = time.monotonic()
    
    # Control loop (running at 10 Hz / 0.1s interval)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 5.0:
        time.sleep(0.1)
        
        # Calculate actual time delta
        now = time.monotonic()
        dt = now - last_time
        last_time = now
        
        # 4. Read encoders
        curr_counts = [left_enc.read(), right_enc.read()]
        
        # Calculate tick delta (handling 32-bit overflows)
        delta_L = curr_counts[0] - last_counts[0]
        delta_R = curr_counts[1] - last_counts[1]
        
        if delta_L > 2000000000: delta_L -= 4294967296
        elif delta_L < -2000000000: delta_L += 4294967296
        if delta_R > 2000000000: delta_R -= 4294967296
        elif delta_R < -2000000000: delta_R += 4294967296
        
        last_counts = curr_counts
        
        # Calculate actual wheel output RPM
        enc_rpm_L = (delta_L / (CPR * dt)) * 60.0 * LEFT_MOTOR_DIR
        enc_rpm_R = (delta_R / (CPR * dt)) * 60.0 * RIGHT_MOTOR_DIR
        
        # 5. Send target RPM and actual encoder feedback to the driver!
        # The driver runs the inner PI loop internally.
        motors.set_motors(cmd_rpm_L, cmd_rpm_R, enc_l=enc_rpm_L, enc_r=enc_rpm_R, dt=dt)
        
        print(f"Cmd: L={cmd_rpm_L:+.1f} R={cmd_rpm_R:+.1f} | Actual: L={enc_rpm_L:+.1f} R={enc_rpm_R:+.1f}")

    # 6. Stop and de-energize
    print("Test finished. Stopping UGV...")
    motors.stop()
    motors.deenergize_system()

if __name__ == "__main__":
    main()
