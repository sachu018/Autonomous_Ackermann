#!/usr/bin/env python3
# collect_waypoints.py
# Helper utility to record raw RTK GPS coordinates from the TCP stream while driving manually.

import time
import sys
import os
import csv
import signal

# Add local path to import receiver
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from rtk_receiver import RTKReceiver
import config

class WaypointCollector:
    def __init__(self, filename="raw_drive_log.csv"):
        self.filename = filename
        self.filepath = os.path.join(config.LOG_DIR, filename)
        
        # Ensure log folder exists
        os.makedirs(config.LOG_DIR, exist_ok=True)
        
        # Open CSV file
        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)
        
        # Write standard columns
        self.writer.writerow([
            "UTC", "Fix_Status", "Latitude", "Longitude", "Altitude",
            "Sats_Used", "Sats_View", "HDOP", "Speed_Knots", "Heading_Deg",
            "Continuous_Heading", "RTCM_Bytes"
        ])
        
        self.receiver = RTKReceiver()
        self.running = True
        
        # Handle graceful shutdown
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)
        
        print(f"[Collector] Initialized. Logging raw RTK data to: {self.filepath}")
        print("[Collector] Connecting to stream... (Waiting for RTK Fix)")

    def _sig_handler(self, sig, frame):
        print("\n[Collector] Stop signal received. Saving and closing log file...")
        self.running = False

    def start(self):
        last_logged_utc = ""
        
        while self.running:
            # Sleep at loop rate
            time.sleep(0.05)
            
            if not self.receiver.connected:
                continue
                
            if self.receiver.new_data_available:
                data = self.receiver.get_latest_pose()
                
                # Check for duplicate timestamps (updates at 5 Hz)
                if data["utc"] == last_logged_utc:
                    continue
                    
                last_logged_utc = data["utc"]
                
                # Print status line
                print(f"Log: UTC={data['utc']} | Fix={data['fix_status']} | Lat={data['lat']:.8f} | Lon={data['lon']:.8f} | Sats={data['sats_used']}", end='\r')
                
                # Log raw columns
                self.writer.writerow([
                    data["utc"],
                    data["fix_status"],
                    data["lat"],
                    data["lon"],
                    data["alt"],
                    data["sats_used"],
                    data["sats_view"],
                    data["hdop"],
                    data["speed"],
                    data["heading"],
                    data["continuous_heading"],
                    data["rtcm_bytes"]
                ])
                self.file.flush()
                
        # Cleanup
        self.receiver.close()
        self.file.close()
        print(f"\n✅ Logging complete. File saved to: {self.filepath}")

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "raw_drive_log.csv"
    collector = WaypointCollector(name)
    collector.start()
