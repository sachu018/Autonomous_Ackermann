#!/usr/bin/env python3
# main.py
# Coordinating script for BeagleBone Black closed-loop UGV control.

import os
os.environ["BLINKA_FORCEBOARD"] = "BEAGLEBONE_BLACK"

import time
import math
import signal
import sys
import socket
import json

import config
from flysky_receiver import FlySkyReceiver
from rtk_receiver import RTKReceiver
from localization import Localization
from path_manager import PathManager
from p_controller import WaypointController
from differential_drive import body_to_wheel_rpm
from motor_controller import MotorController
from safety import SafetySupervisor
from logger import UGVLogger

# Loop frequency (Hz)
LOOP_HZ = 20
DT = 1.0 / LOOP_HZ

# Contactor Debounce Settings
CONTACTOR_DEBOUNCE_S = 0.5

# UDP Telemetry settings (for dashboard compatibility)
TELEMETRY_ADDR = ("127.0.0.1", 5005)

class UGVSystem:
    def __init__(self):
        print("[Main] Initializing UGV System modules...")
        self.receiver = FlySkyReceiver()
        self.motors = MotorController()
        
        from hardware import BBBEncoders
        self.encoders = BBBEncoders()
        
        self.rtk = RTKReceiver()
        self.localization = Localization()
        self.safety = SafetySupervisor()
        
        # Interactive Controller Selection
        print("\n==============================================")
        print("      UGV CLOSED-LOOP CONTROLLER SELECTION    ")
        print("==============================================")
        print("  1. Proportional Straight Line Controller (P_control -> straight_line_path.csv)")
        print("  2. Integral Sliding Mode Controller (ISMC -> straight_line_path.csv)")
        print("  3. Rectangle P-Controller (P_control_rect -> rectangle_path.csv)")
        print("  4. Rectangle ISMC Controller (ISMC_rect -> rectangle_path.csv)")
        print("==============================================")
        
        choice = ""
        while choice not in ["1", "2", "3", "4"]:
            try:
                choice = input("Select trajectory controller (1, 2, 3, or 4): ").strip()
            except (KeyboardInterrupt, SystemExit):
                sys.exit(0)
            except Exception:
                pass
                
        if choice == "1":
            self.controller_type = "P_control"
            self.controller = WaypointController()
            self.path_csv = os.path.join(config.BASE_DIR, "straight_line_path.csv")
            if not os.path.exists(self.path_csv):
                self.path_csv = config.WAYPOINT_CSV
        elif choice == "2":
            self.controller_type = "ISMC"
            from ismc_controller import ISMCController
            self.controller = ISMCController()
            self.path_csv = os.path.join(config.BASE_DIR, "straight_line_path.csv")
            if not os.path.exists(self.path_csv):
                self.path_csv = config.WAYPOINT_CSV
        elif choice == "3":
            self.controller_type = "P_control_rect"
            from p_controller_rect import WaypointControllerRect
            self.controller = WaypointControllerRect()
            self.path_csv = os.path.join(config.BASE_DIR, "rectangle_path.csv")
        else:
            self.controller_type = "ISMC_rect"
            from ismc_rect import ISMCControllerRect
            self.controller = ISMCControllerRect()
            self.path_csv = os.path.join(config.BASE_DIR, "rectangle_path.csv")
            
        print(f"[Main] Activated {self.controller_type} controller using '{os.path.basename(self.path_csv)}'.\n")
        
        # Initialize logger on startup
        self.logger = UGVLogger(self.controller_type)
        
        # Load waypoints path file
        self.path_manager = PathManager(self.path_csv)
        
        # Initialize localization yaw to the heading of the first path segment
        if len(self.path_manager.waypoints) >= 2:
            x0, y0 = self.path_manager.waypoints[0][0], self.path_manager.waypoints[0][1]
            x1, y1 = self.path_manager.waypoints[1][0], self.path_manager.waypoints[1][1]
            initial_yaw = math.atan2(y1 - y0, x1 - x0)
            self.localization.yaw = initial_yaw
            print(f"[Main] Preset starting yaw to path angle: {math.degrees(initial_yaw):.1f}°")
        
        # UDP Telemetry Socket
        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Contactor state tracking
        self.prev_swd_on = False
        self.last_contactor_change = 0.0
        self.first_gps_after_arming = True
        
        # Motion variables (current commanded state)
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.cmd_rpm_L = 0.0
        self.cmd_rpm_R = 0.0
        self.state = "DISARMED"
        
        # Setup signals
        signal.signal(signal.SIGINT, self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)
        
        print("[Main] Initialization complete. System is ready.")
        print("[Main] Manual Override Mode active by default. Flip SWD (CH6) to arm.")

    def _handle_contactor(self, swd_on: bool):
        """Monitors SWD switches to open/close contactor safely with debounce."""
        now = time.monotonic()
        rising = swd_on and not self.prev_swd_on
        falling = not swd_on and self.prev_swd_on
        
        if rising or falling:
            elapsed = now - self.last_contactor_change
            if elapsed < CONTACTOR_DEBOUNCE_S:
                # Ignore bouncing switches
                pass
            else:
                if rising:
                    self.motors.energize_system()
                    # Reset waypoint index and sliding integral state
                    self.path_manager.reset()
                    if hasattr(self.controller, "reset"):
                        self.controller.reset()
                    
                    # Re-initialize localization yaw to starting path heading
                    if len(self.path_manager.waypoints) >= 2:
                        x0, y0 = self.path_manager.waypoints[0][0], self.path_manager.waypoints[0][1]
                        x1, y1 = self.path_manager.waypoints[1][0], self.path_manager.waypoints[1][1]
                        initial_yaw = math.atan2(y1 - y0, x1 - x0)
                        self.localization.yaw = initial_yaw
                        
                    self.first_gps_after_arming = True
                    print("[Main] System armed. Resetting path tracker, controller integrals, and heading.")
                else:
                    self.motors.deenergize_system()
                self.last_contactor_change = now
                
        self.prev_swd_on = swd_on

    def _send_dashboard_telemetry(self, x: float, y: float, yaw: float, fix_status: str, 
                                  Xn: float, Yn: float, rc_ok: bool, swd_on: bool):
        """Sends non-blocking telemetry JSON to the local visualizer process."""
        try:
            telemetry_data = {
                "state": self.state,
                "Xn": round(Xn, 2),
                "Yn": round(Yn, 2),
                "fwd_safe_pct": 1.0,  # Standard limit
                "rev_safe_pct": 1.0,
                "swd_on": swd_on,
                "rc_ok": rc_ok,
                "cmd_rpm_l": round(self.cmd_rpm_L, 1),
                "cmd_rpm_r": round(self.cmd_rpm_R, 1),
                "cmd_thr_l": int(self.motors.last_dac_L),
                "cmd_thr_r": int(self.motors.last_dac_R),
                "enc_l": round(self.encoders.rpms["L"], 1),
                "enc_r": round(self.encoders.rpms["R"], 1),
                # Fused Position (UTM relative to dashboard origin)
                "Xn_utm": round(x, 2) if x is not None else 0.0,
                "Yn_utm": round(y, 2) if y is not None else 0.0,
                "f_yaw": round(math.degrees(yaw), 1),
                "f_roll": 0.0,
                "f_pitch": 0.0,
                "c_sys": 3 if fix_status == "RTK FIXED" else (2 if fix_status == "RTK FLOAT" else 0),
                "c_gyro": 3,
                "c_acc": 3,
                "c_mag": 3,
                "temp": 25.0
            }
            self.telemetry_sock.sendto(json.dumps(telemetry_data).encode('utf-8'), TELEMETRY_ADDR)
        except Exception:
            pass

    def run(self):
        """Main 20 Hz execution loop."""
        print("[Main] E-scooter controllers running in open-loop. RTK closed-loop active.")
        
        while True:
            t0 = time.monotonic()
            
            # 1. Read RC transmitter commands
            Xn, Yn, swd_on, rc_mode, fwd_speed_limit, rev_speed_limit, rc_ok = self.receiver.read()
            
            # 2. Manage power contactor relay based on SWD switch state
            self._handle_contactor(swd_on)
            
            # 3. Check motor permissions
            motors_permitted = self.motors.armed and rc_ok
            
            # 4. Multi-Rate loop bridging: check for new RTK updates (approx 5 Hz)
            if self.rtk.new_data_available:
                rtk_pose = self.rtk.get_latest_pose()
                
                # Update localization if RTK coordinates are valid
                if rtk_pose["lat"] != 0.0 and rtk_pose["lon"] != 0.0:
                    lat = rtk_pose["lat"]
                    lon = rtk_pose["lon"]
                    alt = rtk_pose["alt"]
                    fix_status = rtk_pose["fix_status"]
                    utc = rtk_pose["utc"]
                    timestamp = rtk_pose["timestamp"]
                    
                    # Feed raw coordinates into the localization pipeline
                    coord_accepted = self.localization.update(lat, lon, rtk_heading_deg=rtk_pose["heading"])
                    
                    if coord_accepted:
                        x, y, yaw, zone = self.localization.get_pose()
                        
                        # Startup calibration: align Waypoint 0 and initial heading to the UGV's actual position
                        if self.first_gps_after_arming:
                            self.first_gps_after_arming = False
                            if len(self.path_manager.waypoints) >= 2:
                                self.path_manager.align_start_to_actual_position(x, y)
                                curr_wp = self.path_manager.get_current_waypoint()
                                if curr_wp is not None:
                                    target_yaw = math.atan2(curr_wp[1] - y, curr_wp[0] - x)
                                    self.localization.yaw = target_yaw
                                    yaw = target_yaw
                                    print(f"[Main] Startup calibration: Aligned starting heading to match target segment ({math.degrees(target_yaw):.1f}°)")
                        
                        # Update path tracking manager
                        self.path_manager.update(x, y)
                        
                        # Run proportional controller guidance algorithm
                        if not self.path_manager.is_mission_finished():
                            current_wp = self.path_manager.get_current_waypoint()
                            prev_wp = self.path_manager.get_previous_waypoint()
                            next_wp = self.path_manager.get_next_waypoint()
                            
                            # Calculate desired linear speed (V) and angular turn rate (W)
                            try:
                                self.cmd_v, self.cmd_w = self.controller.update(x, y, yaw, current_wp, prev_wp, next_wp)
                            except TypeError:
                                self.cmd_v, self.cmd_w = self.controller.update(x, y, yaw, current_wp, prev_wp)
            
            # 5. Extract current localization state
            x, y, yaw, zone = self.localization.get_pose()
            fix_status = self.rtk.latest_data["fix_status"]
            utc = self.rtk.latest_data["utc"]
            alt = self.rtk.latest_data["alt"]
            lat = self.rtk.latest_data["lat"]
            lon = self.rtk.latest_data["lon"]
            timestamp = self.rtk.latest_data["timestamp"]
            
            # 6. Execute Motion Pipeline State Machine
            if not motors_permitted:
                # System is disarmed
                self.state = "DISARMED" if not self.motors.armed else "NO SIGNAL"
                self.cmd_rpm_L = 0.0
                self.cmd_rpm_R = 0.0
                self.motors.stop()
            else:
                # System is armed and RC connection is active
                # Check for active manual stick deflection (stick override takeover)
                if self.safety.is_stick_deflected(Xn, Yn) or rc_mode == "MANUAL":
                    # --- MANUAL CONTROL MODE ---
                    self.state = "MANUAL"
                    
                    # Apply forward/reverse speed limits from transmitter switches
                    going_fwd = Yn > 0
                    limit_pct = fwd_speed_limit if going_fwd else rev_speed_limit
                    
                    # Linear speed is mapped directly to Yn stick
                    V_manual = Yn * config.V_MAX * limit_pct
                    
                    # Angular turn rate is mapped directly to Xn stick
                    # Apply polynomial speed scaling map to smooth manual steering
                    W_scale = (abs(Xn) ** 3) / (abs(Yn) + 0.6) * 0.6
                    W_manual = -math.copysign(W_scale, Xn) * config.W_MAX * limit_pct
                    
                    # Convert body commands to wheel RPMs
                    self.cmd_rpm_L, self.cmd_rpm_R = body_to_wheel_rpm(V_manual, W_manual)
                    
                    # Drive motors
                    self.motors.set_motors(self.cmd_rpm_L, self.cmd_rpm_R)
                else:
                    # --- AUTONOMOUS MODE (RTK closed-loop tracking) ---
                    # First check for network watchdog timeout (TCP packet loss)
                    if self.safety.check_rtk_loss(timestamp):
                        self.state = "RTK LOST"
                        self.cmd_rpm_L = 0.0
                        self.cmd_rpm_R = 0.0
                        self.motors.brake()
                    elif self.path_manager.is_mission_finished():
                        # Mission completed - halt safely
                        self.state = "FINISHED"
                        self.cmd_rpm_L = 0.0
                        self.cmd_rpm_R = 0.0
                        self.motors.brake()
                        print("\n[Main] Path complete! Stopping vehicle.")
                    else:
                        # Follow waypoints
                        self.state = f"AUTO_WP_{self.path_manager.current_idx}"
                        
                        # Apply speed attenuation based on GPS fix quality
                        speed_multiplier = self.safety.get_speed_multiplier(fix_status)
                        
                        if speed_multiplier <= 0.0:
                            # Lost RTK Fix - brake immediately
                            self.state = "NO RTK FIX"
                            self.cmd_rpm_L = 0.0
                            self.cmd_rpm_R = 0.0
                            self.motors.brake()
                        else:
                            # Apply scale
                            V_auto = self.cmd_v * speed_multiplier
                            W_auto = self.cmd_w
                            
                            # If rotating in place (V=0, W!=0), update kinematic heading estimate
                            if V_auto == 0.0 and abs(W_auto) > 0.01:
                                self.localization.update_in_place_yaw(W_auto, DT)
                            
                            # Convert body commands to wheel RPMs
                            self.cmd_rpm_L, self.cmd_rpm_R = body_to_wheel_rpm(V_auto, W_auto)
                            
                            # Drive motors
                            self.motors.set_motors(self.cmd_rpm_L, self.cmd_rpm_R)
            
            # Read physical wheel encoders
            act_rpm_L, act_rpm_R = self.encoders.update(self.cmd_rpm_L, self.cmd_rpm_R)
            
            # 7. Log data to CSV file at 20 Hz
            if x is not None:
                current_wp = self.path_manager.get_current_waypoint()
                wp_x, wp_y, wp_lat, wp_lon = current_wp if current_wp else (0.0, 0.0, 0.0, 0.0)
                
                self.logger.log_step(
                    utc=utc, lat=lat, lon=lon, alt=alt, 
                    x=self.localization.raw_x, y=self.localization.raw_y,
                    yaw_deg=math.degrees(yaw), wp_idx=self.path_manager.current_idx,
                    wp_x=wp_x, wp_y=wp_y, wp_lat=wp_lat, wp_lon=wp_lon,
                    v=self.cmd_v, w=self.cmd_w,
                    rpm_L=self.cmd_rpm_L, rpm_R=self.cmd_rpm_R,
                    act_rpm_L=act_rpm_L, act_rpm_R=act_rpm_R,
                    dac_L=self.motors.last_dac_L, dac_R=self.motors.last_dac_R,
                    fix_status=fix_status
                )
                
            # 8. Send dashboard telemetry
            self._send_dashboard_telemetry(x, y, yaw, fix_status, Xn, Yn, rc_ok, swd_on)
            
            # 9. Print telemetry summary line
            arm_str = "ARM" if self.motors.armed else "DIS"
            rc_str = "RC:OK" if rc_ok else "RC:LOST"
            gps_pos = f"X:{x:>9.3f} Y:{y:>9.3f} YAW:{math.degrees(yaw):>+5.1f}°" if x is not None else "GPS:WAITING"
            print(
                f"Mode:{rc_mode:<9} State:{self.state:<11} | "
                f"SWD:{arm_str} {rc_str} | "
                f"{gps_pos} | "
                f"Fix:{fix_status:<10} | "
                f"CMD L:{self.cmd_rpm_L:>+5.1f} R:{self.cmd_rpm_R:>+5.1f} rpm",
                end='\r'
            )
            
            # 10. Rate limit execution to LOOP_HZ (20 Hz)
            elapsed = time.monotonic() - t0
            sleep_t = DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _shutdown(self, sig, frame):
        print("\n[Main] Emergency shutdown triggered. Stopping safely...")
        try:
            self.motors.close()
        except Exception:
            pass
        try:
            self.receiver.close()
        except Exception:
            pass
        try:
            self.rtk.close()
        except Exception:
            pass
        try:
            self.logger.close()
        except Exception:
            pass
        print("[Main] Shutdown complete. Exiting.")
        sys.exit(0)

if __name__ == "__main__":
    system = UGVSystem()
    try:
        system.run()
    except Exception as e:
        print(f"\n[Main] Critical system failure: {e}")
        system._shutdown(None, None)
