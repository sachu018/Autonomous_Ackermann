# logger.py
# High-fidelity network streaming CSV logger for UGV autonomous navigation runs.
# Offloads disk file I/O to Raspberry Pi (192.168.50.1) over Ethernet to eliminate BBB execution lags.

import os
import csv
import time
import socket
from datetime import datetime
from config import LOG_DIR, COORDINATE_SMOOTHING_ALPHA, RTK_HOST

RPI_LOG_PORT = 7000  # UDP port for streaming CSV log data to Raspberry Pi

class UGVLogger:
    def __init__(self, mode_suffix: str = ""):
        self.mode_suffix = mode_suffix
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = f"_{mode_suffix}" if mode_suffix else ""
        self.filename = f"closed_loop_log{suffix}_{timestamp}.csv"
        
        # Local path fallback (written without per-step flush to eliminate flash latency stalls)
        os.makedirs(LOG_DIR, exist_ok=True)
        self.filepath = os.path.join(LOG_DIR, self.filename)
        try:
            self.file = open(self.filepath, "w", newline="")
            self.writer = csv.writer(self.file)
        except Exception:
            self.file = None
            self.writer = None
            
        # Non-blocking UDP Socket targeting Raspberry Pi over Ethernet
        self.rpi_host = RTK_HOST  # "192.168.50.1"
        self.rpi_port = RPI_LOG_PORT
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)
        
        header = [
            "Time_Sec", "UTC", "Latitude", "Longitude", "Smoothed_Latitude", "Smoothed_Longitude",
            "UTM_X", "UTM_Y", "UTM_Z", "Smoothed_UTM_X", "Smoothed_UTM_Y", "Yaw_Deg",
            "Target_WP_Index", "Target_WP_X", "Target_WP_Y", "Target_Latitude", "Target_Longitude",
            "Command_V", "Command_W", "Left_RPM", "Right_RPM", "Actual_Left_RPM", "Actual_Right_RPM",
            "Left_DAC", "Right_DAC", "RTK_Fix_Status"
        ]
        
        if self.writer:
            self.writer.writerow(header)
            
        # Send HEADER frame to RPi over UDP to open log file on RPi
        header_msg = f"HEADER,{self.filename}," + ",".join(header)
        self._send_udp(header_msg)
        
        # EMA filter state cache for online smoothing
        self.alpha = COORDINATE_SMOOTHING_ALPHA
        self.smoothed_lat = None
        self.smoothed_lon = None
        self.smoothed_x = None
        self.smoothed_y = None
        
        self.start_time = time.monotonic()
        print(f"[Logger] Initialized. Streaming live logs to Raspberry Pi ({self.rpi_host}:{self.rpi_port}) & local {self.filepath}")

    def _send_udp(self, msg: str):
        try:
            self.udp_sock.sendto(msg.encode('utf-8'), (self.rpi_host, self.rpi_port))
        except Exception:
            pass  # Non-blocking send ignore drops to guarantee 0.0ms lag on BBB

    def log_step(self, utc: str, lat: float, lon: float, alt: float, x: float, y: float, 
                 yaw_deg: float, wp_idx: int, wp_x: float, wp_y: float, wp_lat: float, wp_lon: float,
                 v: float, w: float, rpm_L: float, rpm_R: float, act_rpm_L: float, act_rpm_R: float,
                 dac_L: int, dac_R: int, fix_status: str):
        """
        Logs a single step. Applies online EMA smoothing on raw GPS coordinates.
        Sends data immediately over non-blocking UDP to RPi.
        """
        if self.smoothed_lat is None:
            self.smoothed_lat = lat
            self.smoothed_lon = lon
            self.smoothed_x = x
            self.smoothed_y = y
        else:
            self.smoothed_lat = self.alpha * lat + (1.0 - self.alpha) * self.smoothed_lat
            self.smoothed_lon = self.alpha * lon + (1.0 - self.alpha) * self.smoothed_lon
            self.smoothed_x = self.alpha * x + (1.0 - self.alpha) * self.smoothed_x
            self.smoothed_y = self.alpha * y + (1.0 - self.alpha) * self.smoothed_y

        elapsed_time = time.monotonic() - self.start_time
        
        row_fields = [
            f"{elapsed_time:.3f}", utc, f"{lat:.8f}", f"{lon:.8f}",
            f"{self.smoothed_lat:.8f}", f"{self.smoothed_lon:.8f}",
            f"{x:.3f}", f"{y:.3f}", f"{alt:.2f}", f"{self.smoothed_x:.3f}", f"{self.smoothed_y:.3f}",
            f"{yaw_deg:.1f}", wp_idx, f"{wp_x:.3f}", f"{wp_y:.3f}", f"{wp_lat:.8f}", f"{wp_lon:.8f}",
            f"{v:.3f}", f"{w:.3f}", f"{rpm_L:.1f}", f"{rpm_R:.1f}", f"{act_rpm_L:.1f}", f"{act_rpm_R:.1f}",
            dac_L, dac_R, fix_status
        ]
        
        # 1. Non-blocking UDP packet send to RPi over Ethernet (takes < 0.02 ms)
        data_msg = f"DATA,{self.filename}," + ",".join(str(f) for f in row_fields)
        self._send_udp(data_msg)
        
        # 2. Local buffer write (WITHOUT per-step flush to eliminate flash latency stalls)
        if self.writer:
            try:
                self.writer.writerow(row_fields)
            except Exception:
                pass

    def close(self):
        # Send END frame to RPi over UDP to close log file on RPi
        end_msg = f"END,{self.filename}"
        self._send_udp(end_msg)
        
        if self.file:
            try:
                self.file.flush()
                self.file.close()
                print(f"[Logger] Saved local backup: {self.filepath}")
            except Exception:
                pass
        try:
            self.udp_sock.close()
        except Exception:
            pass
