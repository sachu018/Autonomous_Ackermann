import os
import sys
import csv
import time
import socket
import json
from datetime import datetime

class UGVLogger:
    def __init__(self, log_dir="/home/debian/Robot/logs"):
        os.makedirs(log_dir, exist_ok=True)
        
        # Detect caller script name
        caller_script = os.path.basename(sys.argv[0])
        script_name, _ = os.path.splitext(caller_script)
        if not script_name or script_name == '-c':
            script_name = "ugv_run"
            
        timestamp = datetime.now().strftime("%d%m%y_%H%M")
        self.filename = os.path.join(log_dir, f"{script_name}_{timestamp}.csv")
        
        try:
            self.file = open(self.filename, mode='w', newline='')
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "calendar_time", "relative_time_s", "dt_s",
                "x_fused_m", "y_fused_m", "yaw_fused_deg",
                "cmd_rpm_l", "cmd_rpm_r", "enc_rpm_l", "enc_rpm_r",
                "raw_ticks_l", "raw_ticks_r", "delta_ticks_l", "delta_ticks_r",
                "raw_ax", "raw_ay", "raw_az",
                "raw_gx", "raw_gy", "raw_gz",
                "raw_mx", "raw_my", "raw_mz",
                "x_imu_m", "y_imu_m", "temp_c"
            ])
            self.file.flush()
            print(f"[Logger] Telemetry will be saved to {self.filename}")
        except Exception as e:
            print(f"[Logger] Error initializing log file: {e}")
            self.file = None
            self.writer = None
            
        self.start_time = time.time()
        
        # Setup UDP socket for Dashboard Telemetry
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_addr = ("127.0.0.1", 5005)
        
        # Send instant Handshake packet to mute dashboard's direct I2C polling before calibration
        try:
            handshake = {"online": True, "state": "CALIBRATING"}
            self.sock.sendto(json.dumps(handshake).encode('utf-8'), self.telemetry_addr)
            print("[Logger] UDP handshake sent to dashboard on port 5005.")
        except Exception as e:
            print(f"[Logger] Warning: Failed to send UDP handshake: {e}")
        
    def log_step(self, dt_actual, x_fused, y_fused, yaw_deg, cmd_l, cmd_r, enc_l, enc_r,
                 raw_l, raw_r, delta_l, delta_r, ax, ay, az, gx, gy, gz, mx=0.0, my=0.0, mz=0.0,
                 x_imu=0.0, y_imu=0.0, temp=0.0):
        if self.writer:
            now_cal = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            now_rel = time.time() - self.start_time
            try:
                self.writer.writerow([
                    now_cal, round(now_rel, 3), round(dt_actual, 4),
                    round(x_fused, 4), round(y_fused, 4), round(yaw_deg, 2),
                    round(cmd_l, 1), round(cmd_r, 1), round(enc_l, 1), round(enc_r, 1),
                    raw_l, raw_r, delta_l, delta_r,
                    round(ax, 4), round(ay, 4), round(az, 4),
                    round(gx, 2), round(gy, 2), round(gz, 2),
                    round(mx, 2), round(my, 2), round(mz, 2),
                    round(x_imu, 4), round(y_imu, 4),
                    round(temp, 1)
                ])
                self.file.flush()
            except Exception as e:
                print(f"[Logger] Error writing log row: {e}")
                
        # Broadcast live telemetry over UDP to dashboard listener
        if self.sock:
            packet = {
                "online": True,
                "state": "RUNNING",
                "Xn": round(x_fused, 4), "Yn": round(y_fused, 4),
                "fwd_safe_pct": 1.0, "rev_safe_pct": 1.0,
                "swd_on": False, "rc_ok": True,
                "cmd_rpm_l": round(cmd_l, 1), "cmd_rpm_r": round(cmd_r, 1),
                "cmd_thr_l": 0, "cmd_thr_r": 0,
                "enc_l": round(enc_l, 1), "enc_r": round(enc_r, 1),
                "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                "gx": round(gx, 2), "gy": round(gy, 2), "gz": round(gz, 2),
                "mx": round(mx, 1), "my": round(my, 1), "mz": round(mz, 1),
                "f_yaw": round(yaw_deg, 1), "f_roll": 0.0, "f_pitch": 0.0,
                "temp": round(temp, 1)
            }
            try:
                self.sock.sendto(json.dumps(packet).encode('utf-8'), self.telemetry_addr)
            except Exception:
                pass
            
    def close(self):
        if self.file:
            try:
                self.file.close()
                print(f"[Logger] Log file closed cleanly: {self.filename}")
            except Exception as e:
                print(f"[Logger] Error closing log file: {e}")
        if self.sock:
            # Send offline packet
            try:
                offline = {"online": False, "state": "STANDBY"}
                self.sock.sendto(json.dumps(offline).encode('utf-8'), self.telemetry_addr)
                self.sock.close()
            except Exception:
                pass
