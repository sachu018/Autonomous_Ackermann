# rtk_receiver.py
# Handles TCP socket connection to Raspberry Pi and parses incoming RTK GPS stream in a background thread.

import time
import socket
import threading
from config import RTK_HOST, RTK_PORT, RTK_TIMEOUT_S

class RTKReceiver:
    def __init__(self):
        self.host = RTK_HOST
        self.port = RTK_PORT
        self.lock = threading.Lock()
        
        # Latest telemetry data structure
        self.latest_data = {
            "utc": "",
            "fix_status": "UNKNOWN",
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "sats_used": 0,
            "sats_view": 0,
            "hdop": 99.9,
            "speed": 0.0,
            "heading": 0.0,
            "continuous_heading": 0.0,
            "rtcm_bytes": 0,
            "timestamp": 0.0
        }
        
        self.new_data_available = False
        self.connected = False
        self.running = True
        
        # Start thread
        self.thread = threading.Thread(target=self._run_client, daemon=True)
        self.thread.start()

    def _parse_line(self, line: str) -> bool:
        """Parses the pipe-separated data format broadcasted by the RPi."""
        line_str = line.strip()
        if not line_str:
            return False
            
        fields = {}
        # Expected: UTC: ... | RTK STATUS: ... | LAT: ... | LON: ... | etc.
        for part in line_str.split("|"):
            part = part.strip()
            if ":" in part:
                key, val = part.split(":", 1)
                fields[key.strip().upper()] = val.strip()
                
        if "LAT" not in fields or "LON" not in fields:
            return False
            
        try:
            with self.lock:
                self.latest_data["utc"] = fields.get("UTC", "")
                self.latest_data["fix_status"] = fields.get("RTK STATUS", "UNKNOWN")
                self.latest_data["lat"] = float(fields["LAT"].split()[0])
                self.latest_data["lon"] = float(fields["LON"].split()[0])
                self.latest_data["alt"] = float(fields.get("ALT", "0 m").replace(" m", ""))
                self.latest_data["sats_used"] = int(fields.get("SAT USED", "0"))
                self.latest_data["sats_view"] = int(fields.get("SAT VIEW", "0"))
                self.latest_data["hdop"] = float(fields.get("HDOP", "99.9"))
                
                speed_str = fields.get("SPEED", "0.0")
                self.latest_data["speed"] = float(speed_str.split()[0]) if speed_str else 0.0
                
                hdg_str = fields.get("GNSS HEADING", "0.0")
                self.latest_data["heading"] = float(hdg_str.split()[0]) if hdg_str else 0.0
                
                cont_hdg_str = fields.get("MOTION HEADING", "0.0").replace("°", "")
                self.latest_data["continuous_heading"] = float(cont_hdg_str.split()[0]) if cont_hdg_str else 0.0
                
                self.latest_data["rtcm_bytes"] = int(fields.get("RTCM BYTES", "0"))
                self.latest_data["timestamp"] = time.monotonic()
                
                self.new_data_available = True
                
            return True
        except Exception as e:
            return False

    def _run_client(self):
        """Socket client worker thread."""
        while self.running:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(3.0)
            
            print(f"[RTK] Connecting to RTK Pi Server at {self.host}:{self.port}...")
            self.connected = False
            
            try:
                sock.connect((self.host, self.port))
                sock.settimeout(None)  # Set to blocking for stream reading
                self.connected = True
                print("[RTK] Connected to RTK data stream!")
                
                # Use makefile wrapper for easy line buffer splits
                sock_file = sock.makefile('r', encoding='utf-8', errors='ignore')
                
                for line in sock_file:
                    if not self.running:
                        break
                    line = line.strip()
                    if line:
                        self._parse_line(line)
                        
                sock_file.close()
            except Exception as e:
                print(f"[RTK] Socket connection error: {e}")
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                self.connected = False
                
            if self.running:
                # Retry delay
                time.sleep(2.0)

    def get_latest_pose(self):
        """Reads latest coordinates. Clears new_data_available flag."""
        with self.lock:
            data = dict(self.latest_data)
            self.new_data_available = False
            return data

    def is_healthy(self) -> bool:
        """Returns True if connected and receiving fresh packages (within timeout)."""
        now = time.monotonic()
        with self.lock:
            time_since_last = now - self.latest_data["timestamp"]
            return self.connected and (time_since_last <= RTK_TIMEOUT_S)

    def close(self):
        self.running = False
        print("[RTK] Shutting down RTK receiver worker thread...")
