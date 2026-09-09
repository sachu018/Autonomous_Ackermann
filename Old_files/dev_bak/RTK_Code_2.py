import socket
import csv
import os
from datetime import datetime

# ---------------- SETTINGS ----------------

HOST = "192.168.50.1"
PORT = 6000

SAVE_FOLDER = "/home/debian/Robot/RTK_CSV_log"

# ------------------------------------------

os.makedirs(SAVE_FOLDER, exist_ok=True)

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filepath = os.path.join(SAVE_FOLDER, f"rtk_log_{timestamp}.csv")

print("Saving CSV to:")
print(filepath)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

print("Connected to RTK data stream")

with open(filepath, "w", newline="", buffering=1) as f:

    writer = csv.writer(f)

    writer.writerow([
        "UTC",
        "Fix",
        "Latitude",
        "Longitude",
        "Altitude",
        "Satellites_Used",
        "Satellites_View",
        "HDOP",
        "Speed_knots",
        "Heading_deg",
        "Continuous_heading",
        "RTCM_bytes"
    ])

    # TCP buffer
    buffer = ""

    while True:

        chunk = sock.recv(1024).decode(errors="ignore")

        if not chunk:
            break

        buffer += chunk

        while "\n" in buffer:

            line, buffer = buffer.split("\n", 1)

            line = line.strip()

            if not line:
                continue

            print("LOG:", line)

            # ---------------- Parse received log ----------------

            fields = {}

            for part in line.split("|"):

                part = part.strip()

                if ":" in part:
                    key, value = part.split(":", 1)
                    fields[key.strip()] = value.strip()

            writer.writerow([
                fields.get("UTC", ""),
                fields.get("RTK STATUS", ""),
                fields.get("LAT", ""),
                fields.get("LON", ""),
                fields.get("ALT", "").replace(" m", ""),
                fields.get("SAT USED", ""),
                fields.get("SAT VIEW", ""),
                fields.get("HDOP", ""),
                fields.get("SPEED", ""),
                fields.get("GNSS HEADING", ""),
                fields.get("MOTION HEADING", "").replace("°", ""),
                fields.get("RTCM BYTES", "")
            ])

            f.flush()
            os.fsync(f.fileno())