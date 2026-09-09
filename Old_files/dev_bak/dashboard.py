#!/usr/bin/env python3
"""
visualize_imu.py
----------------
Starts a multi-threaded Python web server on the BeagleBone Black.
Serves a premium, light-mode web dashboard (Tailwind CSS) featuring:
  - Real-time 3D orientation of the UGV chassis using a glTF/GLB model
    (Three.js + GLTFLoader), with manual 360° camera-orbit controls.
  - Real-time scrolling telemetry charts using Chart.js.
  - Live numerical display for calibrated Accel, Gyro and Mag readings.
  - One-click trigger to run IMU calibration.

REQUIRED LOCAL ASSETS (place these under /home/debian/Robot/www/):
  www/js/three.module.min.js   - Three.js ES module build
  www/js/GLTFLoader.js          - Three.js glTF loader (ES module)
  www/js/BufferGeometryUtils.js - GLTFLoader's only dependency
  www/js/chart.js               - Chart.js (unchanged)
  www/models/car_low_poly.glb            - Your UGV chassis model (add manually)
"""

import time
import json
import sys
import os
import subprocess
import socket
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# ── Thread-Safe Global State for Telemetry ────────────────────────────────────────
latest_telemetry_lock = threading.Lock()
latest_telemetry_data = {
    "online": False,
    "state": "STANDBY",
    "Xn": 0.0, "Yn": 0.0,
    "fwd_safe_pct": 1.0, "rev_safe_pct": 1.0,
    "swd_on": False, "rc_ok": False,
    "cmd_rpm_l": 0.0, "cmd_rpm_r": 0.0,
    "cmd_thr_l": 0, "cmd_thr_r": 0,
    "enc_l": 0.0, "enc_r": 0.0,
    # IMU
    "ax": 0.0, "ay": 0.0, "az": -1.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "temp": 0.0
}
last_packet_time = 0.0

def telemetry_source_thread():
    global last_packet_time, latest_telemetry_data
    
    # ── MPU-6050 Register Constants ──
    MPU_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0
    CALIBRATION_FILE = "/home/debian/Robot/imu_calibration.json"

    # Initialize UDP socket (non-blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    try:
        sock.bind(("127.0.0.1", 5005))
        print("[Standalone Server] Bound UDP listener to port 5005 successfully.")
    except Exception as e:
        print(f"[Standalone Server] Warning: Failed to bind UDP listener on port 5005: {e}")
        
    # Initialize BNO055 directly on I2C Bus 2
    bno_sensor = None
    imu_initialized = False
    try:
        import board
        import adafruit_bno055
        i2c_bus = board.I2C()
        bno_sensor = adafruit_bno055.BNO055_I2C(i2c_bus)
        imu_initialized = True
        print("[Standalone Server] Connected directly to BNO055 on I2C Bus 2.")
    except Exception as e:
        print(f"[Standalone Server] Warning: Could not connect to BNO055 directly: {e}")

    # Load calibration parameters
    cal_offsets = {
        "accel_offset_x": 0.0, "accel_offset_y": 0.0, "accel_offset_z": 0.0,
        "gyro_offset_x": 0.0, "gyro_offset_y": 0.0, "gyro_offset_z": 0.0
    }
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                cal = json.load(f)
            cal_offsets.update(cal)
        except Exception:
            pass

    def read_raw_word_local(reg):
        high = i2c_bus.read_byte_data(MPU_ADDR, reg)
        low = i2c_bus.read_byte_data(MPU_ADDR, reg + 1)
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    print("[Standalone Server] Standing by for telemetry packets or reading IMU directly...")
    while True:
        udp_received = False
        try:
            data, addr = sock.recvfrom(2048)
            packet = json.loads(data.decode('utf-8'))
            with latest_telemetry_lock:
                latest_telemetry_data = packet
                latest_telemetry_data["online"] = True
                last_packet_time = time.monotonic()
            udp_received = True
        except BlockingIOError:
            pass # No UDP data available
        except Exception:
            pass
            
        # If no UDP telemetry packets have been received in the last 1.5 seconds,
        # directly poll the MPU-6050 over I2C to keep the visualization active.
        if not udp_received and (time.monotonic() - last_packet_time > 1.5):
            if imu_initialized and bno_sensor is not None:
                try:
                    accel = bno_sensor.acceleration
                    gyro = bno_sensor.gyro
                    mag = bno_sensor.magnetic
                    euler = bno_sensor.euler
                    temp = bno_sensor.temperature
                    cal = bno_sensor.calibration_status
                    
                    ax, ay, az = 0.0, 0.0, -1.0
                    gx, gy, gz = 0.0, 0.0, 0.0
                    mx, my, mz = 0.0, 0.0, 0.0
                    f_yaw, f_roll, f_pitch = 0.0, 0.0, 0.0
                    temp_c = 0.0
                    c_sys, c_gyro, c_acc, c_mag = 0, 0, 0, 0

                    if accel is not None and accel[0] is not None:
                        ax, ay, az = [val / 9.80665 for val in accel]
                    if gyro is not None and gyro[0] is not None:
                        gx, gy, gz = [val * 57.29577951308232 for val in gyro]
                    if mag is not None and mag[0] is not None:
                        mx, my, mz = mag
                    if euler is not None and euler[0] is not None:
                        f_yaw, f_roll, f_pitch = euler
                    if temp is not None:
                        temp_c = temp
                    if cal is not None:
                        c_sys, c_gyro, c_acc, c_mag = cal

                    with latest_telemetry_lock:
                        latest_telemetry_data = {
                            "online": False, 
                            "state": "STANDBY",
                            "Xn": 0.0, "Yn": 0.0,
                            "fwd_safe_pct": 1.0, "rev_safe_pct": 1.0,
                            "swd_on": False, "rc_ok": False,
                            "cmd_rpm_l": 0.0, "cmd_rpm_r": 0.0,
                            "cmd_thr_l": 0, "cmd_thr_r": 0,
                            "enc_l": 0.0, "enc_r": 0.0,
                            "ax": round(ax, 4), "ay": round(ay, 4), "az": round(az, 4),
                            "gx": round(gx, 2), "gy": round(gy, 2), "gz": round(gz, 2),
                            "mx": round(mx, 1), "my": round(my, 1), "mz": round(mz, 1),
                            "f_yaw": round(f_yaw, 1), "f_roll": round(f_roll, 1), "f_pitch": round(f_pitch, 1),
                            "c_sys": c_sys, "c_gyro": c_gyro, "c_acc": c_acc, "c_mag": c_mag,
                            "temp": round(temp_c, 1)
                        }
                except Exception:
                    pass
            else:
                with latest_telemetry_lock:
                    latest_telemetry_data["online"] = False

        time.sleep(0.02)

def get_latest_telemetry():
    with latest_telemetry_lock:
        if time.monotonic() - last_packet_time > 1.5:
            latest_telemetry_data["online"] = False
        return latest_telemetry_data.copy()

HTML_CONTENT = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Agri-Rover</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;700;800&family=Space+Grotesk:wght@400;600;700;800&display=swap" rel="stylesheet">
<script src="/js/chart.js"></script>
<style>
:root{
  --page:#f1f2f4; --card:#ffffff; --ink-900:#0b0d10; --ink-700:#3a3f47; --ink-500:#7a818c; --ink-300:#c7cbd1;
  --brand:#1fc45c; --brand-50:#eefdf3; --brand-700:#0f7c39; --danger:#ef4444; --amber:#f5a524;
}
*{box-sizing:border-box;margin:0;padding:0}
.hidden{display:none}
body{background:var(--page);color:var(--ink-900);font-family:'Outfit',sans-serif;-webkit-font-smoothing:antialiased}
.font-display{font-family:'Space Grotesk',sans-serif}
.shadow-card{box-shadow:0 1px 2px rgba(15,18,22,.04),0 8px 24px rgba(15,18,22,.04)}
.rounded-4xl{border-radius:28px}
.rounded-3xl{border-radius:1.5rem}
.rounded-2xl{border-radius:1rem}
.wrap{max-width:1600px;margin:0 auto}
.axis-row-x{border-left:3px solid #ef4444}
.axis-row-y{border-left:3px solid #1fc45c}
.axis-row-z{border-left:3px solid #3b82f6}
@keyframes pulse-dot{0%{box-shadow:0 0 0 0 rgba(31,196,92,.45)}70%{box-shadow:0 0 0 8px rgba(31,196,92,0)}to{box-shadow:0 0 0 0 rgba(31,196,92,0)}}
.status-pulse{animation:pulse-dot 2s infinite}
@keyframes spin-slow{to{transform:rotate(1turn)}}
.spinner-brand{border:4px solid rgba(15,18,22,.08);border-top-color:var(--brand);animation:spin-slow .9s linear infinite;border-radius:9999px}
.gauge-needle-transition{transition:stroke-dashoffset .25s ease-out}
.icon-btn{width:36px;height:36px;border-radius:9999px;background:var(--page);display:flex;align-items:center;justify-content:center;transition:background-color .15s;border:none;cursor:pointer;color:var(--ink-700)}
.icon-btn:hover{background:#e7e9ec}
.action-btn{display:flex;align-items:center;gap:6px;background:var(--ink-900);color:#fff;font-weight:700;font-size:11px;border-radius:9999px;padding:9px 16px;border:none;cursor:pointer;transition:background-color .15s}
.action-btn:hover{background:#23262b}
.action-btn.ghost{background:var(--page);color:var(--ink-700)}
.action-btn.ghost:hover{background:#e7e9ec}

/* ===== charts row: full width, real height ===== */
.charts-row{display:grid;grid-template-columns:1fr 1fr;gap:1.25rem}
.chart-card{background:var(--card);border-radius:28px;box-shadow:0 1px 2px rgba(15,18,22,.04),0 8px 24px rgba(15,18,22,.04);padding:1.25rem 1.4rem;display:flex;flex-direction:column;height:290px}
.chart-canvas-wrap{flex:1;min-height:0;position:relative;margin-top:.5rem}

/* ===== bottom strip: single row of live-only cards ===== */
.strip{display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:1.25rem}
.strip>*{grid-column:span 3}
.strip .gauge-cell{grid-column:span 2}
.strip .encoders-cell{grid-column:span 2}
.strip .alert-cell{grid-column:span 2}

@media (max-width:1023px){
  .charts-row{grid-template-columns:1fr}
  .strip{grid-template-columns:1fr 1fr}
  .strip>*{grid-column:span 1 !important}
}
@media (max-width:639px){
  .strip{grid-template-columns:1fr}
}
</style>
</head>
<body>

  <!-- ============ HEADER ============ -->
  <header class="wrap" style="display:flex;align-items:center;justify-content:space-between;padding:20px 24px;gap:16px;flex-wrap:wrap">
    <div>
      <h1 class="font-display" style="font-weight:700;font-size:20px;letter-spacing:-.02em">Agri-Rover</h1>
      <p style="font-size:11px;color:var(--ink-500);margin-top:2px;font-weight:600">Dashboard | SATCARD IIT PKD</p>
    </div>
    <div style="display:flex;align-items:center;gap:10px">
      <div style="display:flex;align-items:center;gap:8px;background:#fff;border-radius:9999px;padding:8px 16px 8px 12px" class="shadow-card">
        <span id="status-dot" style="width:8px;height:8px;border-radius:9999px;background:var(--danger);display:inline-block"></span>
        <span id="status-text" style="font-size:11px;font-weight:700;color:var(--ink-700)">DISCONNECTED</span>
      </div>
      <button id="recenter-btn" class="action-btn ghost" aria-label="Recenter view">
        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M22 12h-3M5 12H2"/></svg>
        Recenter
      </button>
      <button onclick="startCalibration()" class="action-btn">
        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M12 2v4M12 18v4M4.93 4.93l2.83 2.83M16.24 16.24l2.83 2.83M2 12h4M18 12h4M4.93 19.07l2.83-2.83M16.24 7.76l2.83-2.83"/></svg>
        Calibrate
      </button>
    </div>
  </header>

  <!-- ============ MAIN ============ -->
  <main class="wrap" style="padding:0 24px 40px;display:flex;flex-direction:column;gap:20px">

    <!-- TOP ROW: orientation panel + 3D viewer -->
    <div style="display:grid;grid-template-columns:1fr;gap:20px" class="top-row">
      <style>
        @media (min-width:1024px){ .top-row{grid-template-columns: 4fr 8fr !important} }
      </style>

      <!-- STATUS / COMPASS PANEL -->
      <section class="shadow-card" style="background:#fff;border-radius:28px;padding:20px;display:flex;flex-direction:column">
        <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:16px">
          <h2 class="font-display" style="font-weight:700;font-size:15px">Orientation</h2>
          <span style="font-size:11px;color:var(--ink-500);font-weight:600">BNO055</span>
        </div>

        <!-- Compass dial -->
        <div style="position:relative;width:100%;max-width:220px;aspect-ratio:1/1;margin:0 auto;border-radius:9999px;background:var(--page);display:flex;align-items:center;justify-content:center">
          <div style="position:absolute;top:-4px;left:50%;transform:translateX(-50%);width:0;height:0;border-left:6px solid transparent;border-right:6px solid transparent;border-top:10px solid var(--danger);z-index:10"></div>
          <div id="compass-dial" style="position:absolute;width:84%;height:84%;border-radius:9999px;border:1px dashed var(--ink-300);transition:transform .1s">
            <span class="font-display" style="position:absolute;top:4px;left:50%;transform:translateX(-50%);font-weight:800;font-size:14px;color:var(--danger)">N</span>
            <span class="font-display" style="position:absolute;right:4px;top:50%;transform:translateY(-50%);font-weight:800;font-size:14px;color:var(--ink-700)">E</span>
            <span class="font-display" style="position:absolute;bottom:4px;left:50%;transform:translateX(-50%);font-weight:800;font-size:14px;color:var(--ink-700)">S</span>
            <span class="font-display" style="position:absolute;left:4px;top:50%;transform:translateY(-50%);font-weight:800;font-size:14px;color:var(--ink-700)">W</span>
          </div>
          <div style="display:flex;flex-direction:column;align-items:center">
            <span id="compass-deg-val" class="font-display" style="font-weight:700;font-size:24px">000°</span>
            <span id="compass-dir-val" style="font-size:12px;font-weight:600;color:var(--ink-500)">N</span>
          </div>
        </div>

        <!-- Roll / Pitch / Yaw -->
        <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:20px;font-size:12px">
          <div class="axis-row-x" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Roll</span>
            <span id="roll-val" class="font-display" style="font-weight:700;font-size:14px">0.0°</span>
          </div>
          <div class="axis-row-y" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Pitch</span>
            <span id="pitch-val" class="font-display" style="font-weight:700;font-size:14px">0.0°</span>
          </div>
          <div class="axis-row-z" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Yaw</span>
            <span id="yaw-val" class="font-display" style="font-weight:700;font-size:14px">0.0°</span>
          </div>
        </div>

        <!-- Mag X/Y/Z -->
        <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:8px;font-size:12px">
          <div class="axis-row-x" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Mag X</span>
            <span id="mag-x-val" class="font-display" style="font-weight:700;font-size:14px">0.0µT</span>
          </div>
          <div class="axis-row-y" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Mag Y</span>
            <span id="mag-y-val" class="font-display" style="font-weight:700;font-size:14px">0.0µT</span>
          </div>
          <div class="axis-row-z" style="background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
            <span style="color:var(--ink-500);font-weight:600">Mag Z</span>
            <span id="mag-z-val" class="font-display" style="font-weight:700;font-size:14px">0.0µT</span>
          </div>
        </div>

        <!-- FlySky RC transmitter -->
        <div style="margin-top:16px">
          <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
            <span style="font-size:9px;font-weight:700;text-transform:uppercase;letter-spacing:.08em;color:var(--ink-500)">FlySky RC</span>
            <span id="flysky-state" class="font-display" style="font-size:10px;font-weight:700;padding:4px 10px;border-radius:9999px;background:var(--page);color:var(--ink-500)">STANDBY</span>
          </div>
          <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:8px;font-size:12px">
            <div style="border-left:3px solid #ef4444;background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
              <span style="color:var(--ink-500);font-weight:600">Stick X</span>
              <span id="stick-x-val" class="font-display" style="font-weight:700;font-size:14px">0.00</span>
            </div>
            <div style="border-left:3px solid #1fc45c;background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
              <span style="color:var(--ink-500);font-weight:600">Stick Y</span>
              <span id="stick-y-val" class="font-display" style="font-weight:700;font-size:14px">0.00</span>
            </div>
            <div style="border-left:3px solid #a855f7;background:var(--page);border-radius:.5rem;padding:8px 10px;display:flex;flex-direction:column">
              <span style="color:var(--ink-500);font-weight:600">Spd Lim</span>
              <span id="speed-lim-val" class="font-display" style="font-weight:700;font-size:14px">100%</span>
            </div>
          </div>
        </div>

        <!-- Status banner -->
        <div id="status-banner" style="margin-top:16px;border-radius:1.5rem;background:rgba(199,203,209,.4);padding:16px 20px;display:flex;flex-direction:column;gap:4px;transition:background-color .3s">
          <span id="banner-sub" style="font-size:12px;font-weight:500;color:rgba(58,63,71,.8)">Heading 000° · N</span>
          <span id="banner-main" class="font-display" style="font-weight:800;font-size:18px;color:var(--ink-700)">STANDBY</span>
        </div>
      </section>

      <!-- 3D VIEWER -->
      <section class="shadow-card" style="background:#fff;border-radius:28px;padding:20px;display:flex;flex-direction:column;position:relative;overflow:hidden">
        <div style="display:flex;align-items:center;justify-content:space-between;margin-bottom:8px">
          <h2 class="font-display" style="font-weight:700;font-size:15px">3D Chassis View</h2>
          <button id="reset-yaw-btn" style="font-size:11px;font-weight:600;color:var(--ink-500);background:var(--page);border:none;border-radius:9999px;padding:6px 12px;cursor:pointer">
            Reset Yaw
          </button>
        </div>
        <div style="flex:1;position:relative;border-radius:1.5rem;overflow:hidden;background:linear-gradient(to bottom,var(--page),#fff);min-height:320px">
          <div id="three-canvas" style="position:absolute;inset:0"></div>
          <div id="model-loading" style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:12px;font-weight:600;color:var(--ink-500)">
            Loading model…
          </div>
        </div>
        <div style="display:flex;align-items:center;justify-content:center;gap:20px;padding-top:16px">
          <button id="orbit-left-btn" class="icon-btn" aria-label="Rotate view left">
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M15 18l-6-6 6-6"/></svg>
          </button>
          <span style="font-size:12px;font-weight:600;color:var(--ink-500)">360°</span>
          <button id="orbit-right-btn" class="icon-btn" aria-label="Rotate view right">
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M9 18l6-6-6-6"/></svg>
          </button>
        </div>
      </section>
    </div>

   

    <!-- BOTTOM STRIP: live-only status cards -->
    <div class="strip">

      <!-- Calibration (bars + level, consolidated — no duplicate list) -->
      <div class="shadow-card" style="background:#fff;border-radius:28px;padding:20px;display:flex;flex-direction:column">
        <div style="display:flex;align-items:baseline;justify-content:space-between">
          <span style="font-size:11px;font-weight:600;color:var(--ink-500)">Calibration</span>
          <span id="cal-overall-pct" class="font-display" style="font-weight:700;font-size:18px">0%</span>
        </div>
        <div style="display:flex;align-items:end;gap:6px;margin-top:12px;height:56px">
          <div style="flex:1;background:rgba(199,203,209,.3);border-radius:.375rem;height:100%;position:relative;overflow:hidden"><div id="cal-bar-sys" style="position:absolute;bottom:0;width:100%;background:var(--danger);border-radius:.375rem;transition:height .3s,background-color .3s;height:0%"></div></div>
          <div style="flex:1;background:rgba(199,203,209,.3);border-radius:.375rem;height:100%;position:relative;overflow:hidden"><div id="cal-bar-gyro" style="position:absolute;bottom:0;width:100%;background:var(--danger);border-radius:.375rem;transition:height .3s,background-color .3s;height:0%"></div></div>
          <div style="flex:1;background:rgba(199,203,209,.3);border-radius:.375rem;height:100%;position:relative;overflow:hidden"><div id="cal-bar-acc" style="position:absolute;bottom:0;width:100%;background:var(--danger);border-radius:.375rem;transition:height .3s,background-color .3s;height:0%"></div></div>
          <div style="flex:1;background:rgba(199,203,209,.3);border-radius:.375rem;height:100%;position:relative;overflow:hidden"><div id="cal-bar-mag" style="position:absolute;bottom:0;width:100%;background:var(--danger);border-radius:.375rem;transition:height .3s,background-color .3s;height:0%"></div></div>
        </div>
        <div style="display:flex;gap:6px;margin-top:6px;font-size:9px;color:var(--ink-500);font-weight:700;text-align:center">
          <span style="flex:1">Sys<br><span id="cal-sys-val">0</span></span>
          <span style="flex:1">Gyro<br><span id="cal-gyro-val">0</span></span>
          <span style="flex:1">Acc<br><span id="cal-acc-val">0</span></span>
          <span style="flex:1">Mag<br><span id="cal-mag-val">0</span></span>
        </div>
        <div style="display:flex;align-items:center;gap:8px;margin-top:12px;flex-wrap:wrap">
          <button id="calibrate-btn" onclick="startAutoCalibration()" style="background:var(--ink-900);color:#fff;border:none;border-radius:9999px;padding:7px 14px;font-family:'Space Grotesk',sans-serif;font-weight:600;font-size:11px;cursor:pointer;transition:opacity .2s" onmouseover="this.style.opacity='0.8'" onmouseout="this.style.opacity='1'">
            Auto Calibrate
          </button>
          <span id="cal-status-msg" style="font-size:10px;color:var(--ink-500)"></span>
        </div>
      </div>

      <!-- Stats -->
      <div class="shadow-card" style="background:#fff;border-radius:28px;padding:20px;display:flex;flex-direction:column;justify-content:space-between">
        <span style="font-size:11px;font-weight:600;color:var(--ink-500)">IMU Stats</span>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:8px">
          <div>
            <span class="font-display" id="stat-total-g" style="font-weight:700;font-size:20px;display:block">0.00<span style="font-size:11px;font-weight:600;color:var(--ink-500)">g</span></span>
            <span style="font-size:10px;color:var(--ink-500);font-weight:500">Total G</span>
          </div>
          <div>
            <span class="font-display" id="stat-gyro-rate" style="font-weight:700;font-size:20px;display:block">0.0<span style="font-size:11px;font-weight:600;color:var(--ink-500)">°/s</span></span>
            <span style="font-size:10px;color:var(--ink-500);font-weight:500">Gyro Rate</span>
          </div>
          <div>
            <span class="font-display" id="stat-temp" style="font-weight:700;font-size:16px;display:block">--°C</span>
            <span style="font-size:10px;color:var(--ink-500);font-weight:500">Temp</span>
          </div>
          <div>
            <span class="font-display" id="stat-total-mag" style="font-weight:700;font-size:16px;display:block">0.0µT</span>
            <span style="font-size:10px;color:var(--ink-500);font-weight:500">Total Mag</span>
          </div>
        </div>
      </div>

      <!-- Wheel speed gauge -->
      <div class="shadow-card gauge-cell" style="background:#fff;border-radius:28px;padding:16px;display:flex;flex-direction:column;align-items:center">
        <span style="font-size:11px;font-weight:600;color:var(--ink-500);align-self:flex-start">Wheel Speed</span>
        <div style="display:flex;align-items:center;gap:6px;margin-top:10px;font-size:10px;font-weight:700">
          <span id="state-rev" style="padding:4px 10px;border-radius:9999px;color:var(--ink-300)">REV</span>
          <span id="state-stby" style="padding:4px 10px;border-radius:9999px;background:var(--ink-900);color:#fff">STBY</span>
          <span id="state-fwd" style="padding:4px 10px;border-radius:9999px;color:var(--ink-300)">FWD</span>
        </div>
        <div style="position:relative;width:100%;max-width:150px;margin-top:12px">
          <svg viewBox="0 0 200 120" style="width:100%">
            <path d="M 10 110 A 90 90 0 0 1 190 110" fill="none" stroke="#eceef1" stroke-width="14" stroke-linecap="round"/>
            <path id="gauge-arc-fill" d="M 10 110 A 90 90 0 0 1 190 110" fill="none" stroke="#1fc45c" stroke-width="14" stroke-linecap="round"
                  stroke-dasharray="282.74" stroke-dashoffset="282.74" class="gauge-needle-transition"/>
          </svg>
          <div style="position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:flex-end;padding-bottom:2px">
            <span id="gauge-rpm-val" class="font-display" style="font-weight:700;font-size:26px;line-height:1">0</span>
            <span style="font-size:10px;font-weight:600;color:var(--ink-500)">/rpm</span>
          </div>
        </div>
      </div>

      <!-- Wheel encoders -->
      <div class="shadow-card encoders-cell" style="background:#fff;border-radius:28px;padding:16px;display:flex;flex-direction:column;justify-content:space-between">
        <span style="font-size:11px;font-weight:600;color:var(--ink-500)">Encoders (RPM)</span>
        <div style="margin-top:10px">
          <div style="display:flex;justify-content:space-between;align-items:baseline">
            <span style="font-size:10px;color:var(--ink-500);font-weight:600">Left</span>
            <span id="enc-l-val" class="font-display" style="font-weight:700;font-size:16px">+0.0</span>
          </div>
          <div style="display:flex;justify-content:space-between;align-items:baseline;margin-top:6px">
            <span style="font-size:10px;color:var(--ink-500);font-weight:600">Right</span>
            <span id="enc-r-val" class="font-display" style="font-weight:700;font-size:16px">+0.0</span>
          </div>
        </div>
      </div>

      <!-- RC / safety alert -->
      <div id="alert-card" class="shadow-card alert-cell" style="background:#fff;border-radius:28px;padding:16px;display:flex;flex-direction:column;justify-content:space-between;transition:background-color .3s">
        <span style="font-size:11px;font-weight:600;color:var(--ink-500)">RC Signal</span>
        <div style="display:flex;align-items:center;gap:8px;margin-top:10px">
          <svg id="alert-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="color:var(--ink-300);flex-shrink:0"><path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>
          <span id="alert-text" class="font-display" style="font-weight:700;font-size:14px">No Signal</span>
        </div>
      </div>

    </div>
     <!-- CHARTS ROW: full width, real size -->
    <div class="charts-row">
      <div class="chart-card">
        <div style="display:flex;align-items:center;justify-content:space-between">
          <span class="font-display" style="font-weight:700;font-size:14px">Accelerometer</span>
          <span id="live-pill" style="font-size:10px;font-weight:700;padding:4px 10px;border-radius:9999px;background:rgba(199,203,209,.3);color:var(--ink-500)">OFFLINE</span>
        </div>
        <div style="display:flex;align-items:center;gap:16px;margin-top:10px;font-size:11px;font-weight:700">
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#ef4444;display:inline-block"></span>X <span id="tele-ax-val" style="color:var(--ink-500)">0.00</span></span>
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#1fc45c;display:inline-block"></span>Y <span id="tele-ay-val" style="color:var(--ink-500)">0.00</span></span>
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#3b82f6;display:inline-block"></span>Z <span id="tele-az-val" style="color:var(--ink-500)">-1.00</span></span>
          <span style="color:var(--ink-300)">·</span>
          <span style="color:var(--ink-500);font-weight:600">g-force</span>
        </div>
        <div class="chart-canvas-wrap"><canvas id="accelChart"></canvas></div>
      </div>

      <div class="chart-card">
        <div style="display:flex;align-items:center;justify-content:space-between">
          <span class="font-display" style="font-weight:700;font-size:14px">Gyroscope</span>
          <span style="font-size:10px;font-weight:700;padding:4px 10px;border-radius:9999px;background:var(--page);color:var(--ink-500)">°/s</span>
        </div>
        <div style="display:flex;align-items:center;gap:16px;margin-top:10px;font-size:11px;font-weight:700">
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#ef4444;display:inline-block"></span>X <span id="tele-gx-val" style="color:var(--ink-500)">0.0</span></span>
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#1fc45c;display:inline-block"></span>Y <span id="tele-gy-val" style="color:var(--ink-500)">0.0</span></span>
          <span style="display:flex;align-items:center;gap:5px"><span style="width:8px;height:8px;border-radius:9999px;background:#3b82f6;display:inline-block"></span>Z <span id="tele-gz-val" style="color:var(--ink-500)">0.0</span></span>
          <span style="color:var(--ink-300)">·</span>
          <span style="color:var(--ink-500);font-weight:600">angular rate</span>
        </div>
        <div class="chart-canvas-wrap"><canvas id="gyroChart"></canvas></div>
      </div>
    </div>
  </main>

  <!-- ============ CALIBRATION MODAL ============ -->
  <div id="cal-modal" class="hidden" style="position:fixed;inset:0;background:rgba(11,13,16,.4);backdrop-filter:blur(4px);z-index:1000;align-items:center;justify-content:center;padding:0 16px">
    <div style="background:#fff;border-radius:28px;padding:32px;max-width:420px;width:100%;text-align:center" class="shadow-card">
      <span class="font-display" style="font-weight:700;font-size:18px">IMU Calibration Active</span>
      <div class="spinner-brand" style="width:48px;height:48px;margin:24px auto"></div>
      <p style="font-size:14px;color:var(--ink-700)">Keep the UGV completely flat and motionless on a level surface.</p>
      <p style="font-size:12px;color:var(--ink-500);margin-top:8px" id="cal-subtext">Sampling 500 calibration points…</p>
      <div style="width:100%;height:6px;background:var(--page);border-radius:9999px;margin-top:20px;overflow:hidden">
        <div id="progress-fill" style="height:100%;background:var(--brand);border-radius:9999px;transition:width .1s;width:0%"></div>
      </div>
    </div>
  </div>

<script type="importmap">
{
  "imports": {
    "three": "/js/three.module.min.js"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { GLTFLoader } from '/js/GLTFLoader.js';

// ===== State =====
let connectionActive = false;
let eventSource = null;
let pitch = 0.0, roll = 0.0, yaw = 0.0;
let lastTimestamp = null;

const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');


const canvasContainer = document.getElementById('three-canvas').parentElement;
const loadingLabel = document.getElementById('model-loading');

const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(40, canvasContainer.clientWidth / canvasContainer.clientHeight, 0.1, 100);
let cameraAngle = Math.PI / 4;
const cameraRadius = 3.4;
const cameraHeight = 1.5;
function placeCamera() {
  camera.position.set(Math.sin(cameraAngle) * cameraRadius, cameraHeight, Math.cos(cameraAngle) * cameraRadius);
  camera.lookAt(0, 0.15, 0);
}
placeCamera();

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setClearColor(0x000000, 0);
renderer.setSize(canvasContainer.clientWidth, canvasContainer.clientHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
document.getElementById('three-canvas').appendChild(renderer.domElement);
scene.add(new THREE.HemisphereLight(0xffffff, 0xd8dee4, 1.15));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.3);
keyLight.position.set(4, 6, 4);
scene.add(keyLight);
const rimLight = new THREE.DirectionalLight(0x1fc45c, 0.35); // subtle brand-green rim light
rimLight.position.set(-4, 2, -3);
scene.add(rimLight);


const modelGroup = new THREE.Group();
scene.add(modelGroup);

let groundShadow = null;
function addGroundShadow(radius, y) {
  const canvas = document.createElement('canvas');
  canvas.width = 256; canvas.height = 256;
  const ctx = canvas.getContext('2d');
  const grad = ctx.createRadialGradient(128, 128, 0, 128, 128, 128);
  grad.addColorStop(0, 'rgba(15,18,22,0.22)');
  grad.addColorStop(1, 'rgba(15,18,22,0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 256, 256);
  const texture = new THREE.CanvasTexture(canvas);
  const mat = new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false });
  const geo = new THREE.CircleGeometry(radius, 48);
  geo.rotateX(-Math.PI / 2);
  groundShadow = new THREE.Mesh(geo, mat);
  groundShadow.position.y = y;
  scene.add(groundShadow);
}const loader = new GLTFLoader();
loader.load(
  '/models/car_low_poly.glb',
  (gltf) => {
    const model = gltf.scene;
    const box = new THREE.Box3().setFromObject(model);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    model.position.sub(center);
    const maxDim = Math.max(size.x, size.y, size.z) || 1;
    const scale = 1.9 / maxDim;
    model.scale.setScalar(scale);
    modelGroup.add(model);

    const scaledBox = new THREE.Box3().setFromObject(modelGroup);
    addGroundShadow(Math.max(scaledBox.max.x - scaledBox.min.x, scaledBox.max.z - scaledBox.min.z) * 0.6, scaledBox.min.y - 0.01);

    loadingLabel.style.display = 'none';
  },
  undefined,
  () => {
    loadingLabel.innerText = 'Could not load /models/car_low_poly.glb — add your model file at that path.';
  }
);

function animate() {
  requestAnimationFrame(animate);
  modelGroup.rotation.x = roll * Math.PI / 180.0;
  modelGroup.rotation.z = -pitch * Math.PI / 180.0;
  modelGroup.rotation.y = yaw * Math.PI / 180.0;
  renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
  const w = canvasContainer.clientWidth, h = canvasContainer.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});


let orbitAnimId = null;
function orbitTo(targetAngle) {
  if (orbitAnimId) cancelAnimationFrame(orbitAnimId);
  const startAngle = cameraAngle;
  const delta = targetAngle - startAngle;
  const duration = 350;
  const startTime = performance.now();
  function step(now) {
    const t = Math.min(1, (now - startTime) / duration);
    const eased = 1 - Math.pow(1 - t, 3);
    cameraAngle = startAngle + delta * eased;
    placeCamera();
    if (t < 1) orbitAnimId = requestAnimationFrame(step);
  }
  orbitAnimId = requestAnimationFrame(step);
}

document.getElementById('orbit-left-btn').addEventListener('click', () => orbitTo(cameraAngle - Math.PI / 4));
document.getElementById('orbit-right-btn').addEventListener('click', () => orbitTo(cameraAngle + Math.PI / 4));
document.getElementById('recenter-btn').addEventListener('click', () => {
  orbitTo(Math.PI / 4);
  resetYaw();
});
document.getElementById('reset-yaw-btn').addEventListener('click', resetYaw);
function resetYaw() { yaw = 0.0; }
window.resetYaw = resetYaw;

// ===========================================================
// 2. Chart.js — full-size live telemetry charts (light theme)
// ===========================================================
const maxPoints = 50;
const chartOptions = {
  responsive: true,
  maintainAspectRatio: false,
  animation: false,
  scales: {
    x: { display: false },
    y: { display: false }
  },
  plugins: { legend: { display: false } },
  elements: { line: { borderJoinStyle: 'round' } }
};

const accelCtx = document.getElementById('accelChart').getContext('2d');
const accelChart = new Chart(accelCtx, {
  type: 'line',
  data: {
    labels: Array(maxPoints).fill(''),
    datasets: [
      { data: Array(maxPoints).fill(0), borderColor: '#ef4444', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 },
      { data: Array(maxPoints).fill(0), borderColor: '#1fc45c', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 },
      { data: Array(maxPoints).fill(-1), borderColor: '#3b82f6', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 }
    ]
  },
  options: chartOptions
});

const gyroCtx = document.getElementById('gyroChart').getContext('2d');
const gyroChart = new Chart(gyroCtx, {
  type: 'line',
  data: {
    labels: Array(maxPoints).fill(''),
    datasets: [
      { data: Array(maxPoints).fill(0), borderColor: '#ef4444', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 },
      { data: Array(maxPoints).fill(0), borderColor: '#1fc45c', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 },
      { data: Array(maxPoints).fill(0), borderColor: '#3b82f6', borderWidth: 2, pointRadius: 0, fill: false, tension: 0.3 }
    ]
  },
  options: chartOptions
});

// ===========================================================
// 3. Calibration helpers
// ===========================================================
const GAUGE_MAX_RPM = 200;
const GAUGE_ARC_LENGTH = 282.74; // π * r(90), exact semicircle

function levelColor(level) {
  if (level >= 3) return '#1fc45c';
  if (level >= 1) return '#f5a524';
  return '#ef4444';
}

function updateCalChannel(key, level) {
  const val = document.getElementById(`cal-${key}-val`);
  const bar = document.getElementById(`cal-bar-${key}`);
  const color = levelColor(level);
  if (val) { val.innerText = level; val.style.color = color; }
  if (bar) { bar.style.height = `${(level / 3) * 100}%`; bar.style.backgroundColor = color; }
}

function getDirection(deg) {
  const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  const index = Math.round(((deg % 360) + 360) % 360 / 45) % 8;
  return directions[index];
}

// ===========================================================
// 4. Calibration trigger (modal flow)
// ===========================================================
function startCalibration() {
  const modal = document.getElementById('cal-modal');
  const progressFill = document.getElementById('progress-fill');
  const subtext = document.getElementById('cal-subtext');

  modal.classList.remove('hidden');
  modal.style.display = 'flex';
  progressFill.style.width = '0%';
  subtext.innerText = 'Connecting to calibration task…';

  if (eventSource) eventSource.close();

  fetch('/calibrate', { method: 'POST' })
    .then((res) => {
      if (res.status === 200) {
        let progress = 0;
        const duration = 13000;
        const intervalTime = 100;
        const increment = (intervalTime / duration) * 100;

        const timer = setInterval(() => {
          progress += increment;
          if (progress >= 100) {
            progress = 100;
            clearInterval(timer);
            subtext.innerText = 'Calibration completed! Saving offsets…';
            setTimeout(() => {
              modal.classList.add('hidden');
              modal.style.display = 'none';
              pitch = 0; roll = 0; yaw = 0;
              lastTimestamp = null;
              connectSSE();
            }, 1000);
          }
          progressFill.style.width = `${progress}%`;
          subtext.innerText = progress < 25
            ? 'Initializing IMU… Keep flat and still.'
            : `Sampling 500 calibration points… (${Math.round((progress - 25) / 75 * 100)}%)`;
        }, intervalTime);
      } else {
        alert('Calibration failed to start. Is the service busy?');
        modal.classList.add('hidden');
        modal.style.display = 'none';
        connectSSE();
      }
    })
    .catch((err) => {
      alert(`Error triggering calibration: ${err}`);
      modal.classList.add('hidden');
      modal.style.display = 'none';
      connectSSE();
    });
}
window.startCalibration = startCalibration;

// ===========================================================
// 4b. Auto Calibrate — non-blocking variant (no modal, just
//     watch the Lvl badges above climb while the robot moves)
// ===========================================================
function startAutoCalibration() {
  const btn = document.getElementById('calibrate-btn');
  const msg = document.getElementById('cal-status-msg');
  if (!btn) return;
  btn.disabled = true;
  btn.style.opacity = '0.5';
  msg.style.color = 'var(--amber)';
  msg.innerText = 'Calibration running… Robot is moving.';

  fetch('/calibrate', { method: 'POST' })
    .then((r) => r.json())
    .then((data) => {
      if (data.status === 'calibration_started') {
        msg.style.color = 'var(--brand-700)';
        msg.innerText = 'Calibration started — watch the levels above.';
        setTimeout(() => {
          btn.disabled = false;
          btn.style.opacity = '1';
          msg.innerText = 'Done. Check levels to confirm.';
        }, 120000);
      }
    })
    .catch((err) => {
      btn.disabled = false;
      btn.style.opacity = '1';
      msg.style.color = 'var(--danger)';
      msg.innerText = `Could not start calibration: ${err}`;
    });
}
window.startAutoCalibration = startAutoCalibration;

// ===========================================================
// 5. Server-Sent Events — live telemetry stream
// ===========================================================
function connectSSE() {
  if (eventSource) eventSource.close();
  eventSource = new EventSource('/events');

  eventSource.onopen = () => {
    connectionActive = true;
    statusDot.style.backgroundColor = 'var(--brand)';
    statusDot.classList.add('status-pulse');
    statusText.innerText = 'CONNECTED';
  };

  eventSource.onerror = () => {
    connectionActive = false;
    statusDot.style.backgroundColor = 'var(--danger)';
    statusDot.classList.remove('status-pulse');
    statusText.innerText = 'DISCONNECTED';
    lastTimestamp = null;
    setTimeout(connectSSE, 3000);
  };

  eventSource.onmessage = (event) => {
    const data = JSON.parse(event.data);

    const ax = data.ax, ay = data.ay, az = data.az;
    const gx = data.gx, gy = data.gy, gz = data.gz;
    const temp = data.temp;
    const mx = data.mx ?? 0.0, my = data.my ?? 0.0, mz = data.mz ?? 0.0;
    const c_sys = data.c_sys ?? 0, c_gyro = data.c_gyro ?? 0, c_acc = data.c_acc ?? 0, c_mag = data.c_mag ?? 0;
    const online = data.online;
    const rc_ok = data.rc_ok;
    const enc_l = data.enc_l ?? 0.0, enc_r = data.enc_r ?? 0.0;
    const state = data.state ?? 'STANDBY';
    const Xn = data.Xn ?? 0.0, Yn = data.Yn ?? 0.0;
    const fwd_safe_pct = data.fwd_safe_pct ?? 1.0, rev_safe_pct = data.rev_safe_pct ?? 1.0;

    const now = Date.now();
    let dt = 0.02;
    if (lastTimestamp) dt = (now - lastTimestamp) / 1000.0;
    lastTimestamp = now;

    if (data.f_yaw !== undefined && data.f_roll !== undefined && data.f_pitch !== undefined) {
      yaw = data.f_yaw; roll = data.f_roll; pitch = data.f_pitch;
    } else {
      const accelPitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180.0 / Math.PI;
      const accelRoll = Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * 180.0 / Math.PI;
      pitch = 0.98 * (pitch + gy * dt) + 0.02 * accelPitch;
      roll = 0.98 * (roll + gx * dt) + 0.02 * accelRoll;
      yaw -= gz * dt;
    }

    // Orientation readout
    document.getElementById('roll-val').innerText = `${roll > 0 ? '+' : ''}${roll.toFixed(1)}°`;
    document.getElementById('pitch-val').innerText = `${pitch > 0 ? '+' : ''}${pitch.toFixed(1)}°`;
    document.getElementById('yaw-val').innerText = `${yaw > 0 ? '+' : ''}${yaw.toFixed(1)}°`;

    document.getElementById('mag-x-val').innerText = `${mx > 0 ? '+' : ''}${mx.toFixed(1)}µT`;
    document.getElementById('mag-y-val').innerText = `${my > 0 ? '+' : ''}${my.toFixed(1)}µT`;
    document.getElementById('mag-z-val').innerText = `${mz > 0 ? '+' : ''}${mz.toFixed(1)}µT`;
    const magField = Math.sqrt(mx * mx + my * my + mz * mz);

    // Compass
    const compassDial = document.getElementById('compass-dial');
    compassDial.style.transform = `rotate(${-yaw}deg)`;
    document.getElementById('compass-deg-val').innerText = `${Math.round(((yaw % 360) + 360) % 360)}°`;
    document.getElementById('compass-dir-val').innerText = getDirection(yaw);

    // Status banner
    const banner = document.getElementById('status-banner');
    const bannerMain = document.getElementById('banner-main');
    const bannerSub = document.getElementById('banner-sub');
    bannerSub.innerText = `Heading ${Math.round(((yaw % 360) + 360) % 360)}° · ${getDirection(yaw)}`;
    if (online && rc_ok) {
      banner.style.background = 'var(--brand-50)';
      bannerMain.style.color = 'var(--brand-700)';
      bannerMain.innerText = 'CONNECTED';
      bannerSub.style.color = 'rgba(15,124,57,.7)';
    } else if (online) {
      banner.style.background = '#fffbeb';
      bannerMain.style.color = '#b45309';
      bannerMain.innerText = 'NO RC SIGNAL';
      bannerSub.style.color = 'rgba(180,83,9,.7)';
    } else {
      banner.style.background = 'rgba(199,203,209,.3)';
      bannerMain.style.color = 'var(--ink-700)';
      bannerMain.innerText = 'STANDBY';
      bannerSub.style.color = 'var(--ink-500)';
    }

    // Stats card
    const gMag = Math.sqrt(ax * ax + ay * ay + az * az);
    const gyroMag = Math.sqrt(gx * gx + gy * gy + gz * gz);
    document.getElementById('stat-total-g').innerHTML = `${gMag.toFixed(2)}<span style="font-size:11px;font-weight:600;color:var(--ink-500)">g</span>`;
    document.getElementById('stat-gyro-rate').innerHTML = `${gyroMag.toFixed(1)}<span style="font-size:11px;font-weight:600;color:var(--ink-500)">°/s</span>`;
    document.getElementById('stat-temp').innerText = temp ? `${temp.toFixed(1)}°C` : '--°C';
    document.getElementById('stat-total-mag').innerText = `${magField.toFixed(1)}µT`;

    // Live telemetry legend + charts
    document.getElementById('tele-ax-val').innerText = ax.toFixed(2);
    document.getElementById('tele-ay-val').innerText = ay.toFixed(2);
    document.getElementById('tele-az-val').innerText = az.toFixed(2);
    document.getElementById('tele-gx-val').innerText = gx.toFixed(1);
    document.getElementById('tele-gy-val').innerText = gy.toFixed(1);
    document.getElementById('tele-gz-val').innerText = gz.toFixed(1);

    accelChart.data.datasets[0].data.push(ax); accelChart.data.datasets[0].data.shift();
    accelChart.data.datasets[1].data.push(ay); accelChart.data.datasets[1].data.shift();
    accelChart.data.datasets[2].data.push(az); accelChart.data.datasets[2].data.shift();
    accelChart.update();
    gyroChart.data.datasets[0].data.push(gx); gyroChart.data.datasets[0].data.shift();
    gyroChart.data.datasets[1].data.push(gy); gyroChart.data.datasets[1].data.shift();
    gyroChart.data.datasets[2].data.push(gz); gyroChart.data.datasets[2].data.shift();
    gyroChart.update();

    const livePill = document.getElementById('live-pill');
    livePill.innerText = online ? 'LIVE' : 'OFFLINE';
    livePill.style.background = online ? 'var(--brand-50)' : 'rgba(199,203,209,.3)';
    livePill.style.color = online ? 'var(--brand-700)' : 'var(--ink-500)';

    // Calibration
    updateCalChannel('sys', c_sys);
    updateCalChannel('gyro', c_gyro);
    updateCalChannel('acc', c_acc);
    updateCalChannel('mag', c_mag);
    const overallPct = Math.round(((c_sys + c_gyro + c_acc + c_mag) / 12) * 100);
    document.getElementById('cal-overall-pct').innerText = `${overallPct}%`;

    // FlySky RC transmitter
    const flyskyState = document.getElementById('flysky-state');
    flyskyState.innerText = `${state} (${online ? 'ACTIVE' : 'STANDBY'})`;
    flyskyState.style.background = online ? (rc_ok ? 'var(--brand-50)' : '#fef2f2') : 'var(--page)';
    flyskyState.style.color = online ? (rc_ok ? 'var(--brand-700)' : 'var(--danger)') : 'var(--ink-500)';
    document.getElementById('stick-x-val').innerText = `${Xn > 0 ? '+' : ''}${Xn.toFixed(2)}`;
    document.getElementById('stick-y-val').innerText = `${Yn > 0 ? '+' : ''}${Yn.toFixed(2)}`;
    const speedLimit = Yn >= 0 ? fwd_safe_pct : rev_safe_pct;
    document.getElementById('speed-lim-val').innerText = `${Math.round(speedLimit * 100)}%`;

    // Wheel encoders + speed gauge
    document.getElementById('enc-l-val').innerText = `${enc_l > 0 ? '+' : ''}${enc_l.toFixed(1)}`;
    document.getElementById('enc-r-val').innerText = `${enc_r > 0 ? '+' : ''}${enc_r.toFixed(1)}`;

    const avgSigned = (enc_l + enc_r) / 2;
    const avgAbs = Math.min(Math.abs(avgSigned), GAUGE_MAX_RPM);
    document.getElementById('gauge-rpm-val').innerText = Math.round(avgAbs);
    const fraction = avgAbs / GAUGE_MAX_RPM;
    document.getElementById('gauge-arc-fill').setAttribute('stroke-dashoffset', `${GAUGE_ARC_LENGTH * (1 - fraction)}`);

    const stateRev = document.getElementById('state-rev');
    const stateStby = document.getElementById('state-stby');
    const stateFwd = document.getElementById('state-fwd');
    [stateRev, stateStby, stateFwd].forEach((el) => { el.style.background = 'transparent'; el.style.color = 'var(--ink-300)'; });
    if (Math.abs(avgSigned) < 2) {
      stateStby.style.background = 'var(--ink-900)'; stateStby.style.color = '#fff';
    } else if (avgSigned > 0) {
      stateFwd.style.background = 'var(--ink-900)'; stateFwd.style.color = '#fff';
    } else {
      stateRev.style.background = 'var(--ink-900)'; stateRev.style.color = '#fff';
    }

    // RC / safety alert card
    const alertCard = document.getElementById('alert-card');
    const alertIcon = document.getElementById('alert-icon');
    const alertText = document.getElementById('alert-text');
    if (online && rc_ok) {
      alertCard.style.background = 'var(--brand-50)';
      alertIcon.style.color = '#16a34a';
      alertText.style.color = 'var(--brand-700)';
      alertText.innerText = 'Signal OK';
    } else if (online) {
      alertCard.style.background = '#fef2f2';
      alertIcon.style.color = 'var(--danger)';
      alertText.style.color = 'var(--danger)';
      alertText.innerText = 'No Signal';
    } else {
      alertCard.style.background = '#fff';
      alertIcon.style.color = 'var(--ink-300)';
      alertText.style.color = 'var(--ink-500)';
      alertText.innerText = 'Standby';
    }

    // Header connection pill
    if (online) {
      statusDot.style.backgroundColor = rc_ok ? 'var(--brand)' : '#f59e0b';
      statusDot.classList.add('status-pulse');
      statusText.innerText = rc_ok ? 'CONNECTED · UGV LOOP ACTIVE' : 'CONNECTED · NO RC SIGNAL';
    } else {
      statusDot.style.backgroundColor = 'var(--ink-300)';
      statusDot.classList.remove('status-pulse');
      statusText.innerText = 'STANDBY';
    }
  };
}

connectSSE();
</script>

</body>
</html>
"""

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class IMURequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/calibrate":
            import subprocess, threading
            def _run():
                subprocess.run(
                    ["sudo", "PYTHONPATH=/home/debian/.local/lib/python3.11/site-packages", "python3", "/home/debian/Robot/auto_calibrate_bno055.py"],
                    stdout=open("/home/debian/Robot/logs/calibration.log", "w"),
                    stderr=subprocess.STDOUT
                )
            threading.Thread(target=_run, daemon=True).start()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"status":"calibration_started"}')
        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.end_headers()

    def log_message(self, format, *args):
        return

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
            
        elif self.path in (
            '/js/three.module.min.js', '/js/GLTFLoader.js',
            '/js/BufferGeometryUtils.js', '/js/chart.js'
        ):
            local_file = "/home/debian/Robot/www" + self.path
            if os.path.exists(local_file):
                self.send_response(200)
                self.send_header('Content-type', 'application/javascript')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                with open(local_file, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()

        elif self.path == '/models/car_low_poly.glb':
            local_file = "/home/debian/Robot/www" + self.path
            if os.path.exists(local_file):
                self.send_response(200)
                self.send_header('Content-type', 'model/gltf-binary')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                with open(local_file, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(404)
                self.end_headers()

        elif self.path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            try:
                while True:
                    data = get_latest_telemetry()
                    event_str = f"data: {json.dumps(data)}\n\n"
                    self.wfile.write(event_str.encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.02)
            except (ConnectionResetError, BrokenPipeError, Exception):
                pass

    def do_POST(self):
        if self.path == '/calibrate':
            print("[Web Server] Triggering calibration subprocess...")
            try:
                proc = subprocess.Popen(["python3", "/home/debian/Robot/calibrate_imu.py", "--auto"])
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "started"}).encode('utf-8'))
                proc.wait()
            except Exception as e:
                print(f"[Web Server] Calibration failed: {e}")
                self.send_response(500)
                self.end_headers()

def main():
    # Start the UDP background listener & Direct I2C Poller thread
    t = threading.Thread(target=telemetry_source_thread, daemon=True)
    t.start()
    
    server_address = ('', 8080)
    httpd = ThreadedHTTPServer(server_address, IMURequestHandler)
    print("\n" + "="*60)
    print("UGV IMU STANDALONE Visualization Server is Running!")
    print("This server runs when UGV robot.service is stopped/off.")
    print("Navigate to: http://192.168.7.2:8080/")
    print("="*60 + "\n")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Web Server] Shutting down...")
    finally:
        httpd.server_close()

if __name__ == '__main__':
    main()
