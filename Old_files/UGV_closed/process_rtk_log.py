#!/usr/bin/env python3
# process_rtk_log.py
# Processes raw logged RTK paths into smoothed, filtered, resampled waypoint files.

import sys
import os
import csv
import math

# WGS-84 UTM Projection Constants
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563

def latlon_to_utm(lat: float, lon: float, force_zone=43):
    """
    Converts latitude/longitude coordinates to UTM Easting, Northing, and Zone.
    Defaults to Zone 43 (Palakkad, Kerala, India) to prevent coordinate jumps.
    """
    b = WGS84_A * (1 - WGS84_F)
    e2 = (WGS84_A**2 - b**2) / WGS84_A**2
    e_prime2 = (WGS84_A**2 - b**2) / b**2
    
    if force_zone is not None:
        zone = force_zone
    else:
        zone = int((lon + 180) / 6) + 1
    lon_origin = (zone - 1) * 6 - 180 + 3
    lon_origin_rad = math.radians(lon_origin)
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    k0 = 0.9996  # Scale factor
    
    N = WGS84_A / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    T = math.tan(lat_rad)**2
    C = e_prime2 * math.cos(lat_rad)**2
    A = (lon_rad - lon_origin_rad) * math.cos(lat_rad)
    
    # Meridional Arc Length
    M = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat_rad
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat_rad)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat_rad)
        - (35 * e2**3 / 3072) * math.sin(6 * lat_rad)
    )
    
    easting = k0 * N * (
        A
        + (1 - T + C) * A**3 / 6
        + (5 - 18 * T + T**2 + 72 * C - 58 * e_prime2) * A**5 / 120
    ) + 500000.0
    
    northing = k0 * (
        M
        + N
        * math.tan(lat_rad)
        * (
            A**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * A**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * e_prime2) * A**6 / 720
        )
    )
    if lat < 0:
        northing += 10000000.0
        
    return easting, northing, zone

def detect_delimiter(file_path):
    """Detects delimiter type (tab, comma, or pipe) based on symbol frequencies."""
    with open(file_path, 'r', errors='ignore') as f:
        first_few_lines = [f.readline() for _ in range(5)]
        
    tabs = sum(line.count('\t') for line in first_few_lines)
    commas = sum(line.count(',') for line in first_few_lines)
    pipes = sum(line.count('|') for line in first_few_lines)
    
    if pipes > max(tabs, commas):
        return '|'
    elif tabs > commas:
        return '\t'
    return ','

def parse_line_dict(line: str, delimiter: str):
    """Parses a line string into a dictionary containing raw GPS fields."""
    line_str = line.strip()
    if not line_str:
        return None
        
    # Format 1: Pipe separated (e.g. UTC: ... | LAT: ... | LON: ...)
    if delimiter == '|':
        data = {}
        parts = line_str.split('|')
        for part in parts:
            if ':' in part:
                k, v = part.split(':', 1)
                data[k.strip().upper()] = v.strip()
        try:
            lat = float(data.get('LAT', '0').split()[0])
            lon = float(data.get('LON', '0').split()[0])
            alt = float(data.get('ALT', '0').split()[0])
            sats_used = int(data.get('SAT USED', '0'))
            sats_view = int(data.get('SAT VIEW', '0'))
            hdop = float(data.get('HDOP', '0'))
            speed = float(data.get('SPEED', '0'))
            
            hdg_str = data.get('MOTION HEADING', '0')
            if '°' in hdg_str:
                hdg_str = hdg_str.replace('°', '')
            heading_cont = float(hdg_str.split()[0])
            
            rtcm = int(data.get('RTCM BYTES', '0'))
            fix = data.get('RTK STATUS', 'UNKNOWN')
            utc = data.get('UTC', '')
            
            return {
                'utc': utc, 'fix': fix, 'lat': lat, 'lon': lon, 'alt': alt,
                'sats_used': sats_used, 'sats_view': sats_view, 'hdop': hdop,
                'speed': speed, 'heading': 0.0, 'heading_cont': heading_cont,
                'rtcm': rtcm
            }
        except Exception:
            return None

    # Format 2 & 3: Tab or Comma-separated columns
    row = next(csv.reader([line_str], delimiter=delimiter))
    
    # Skip headers
    if not row or row[0].strip().lower() == 'utc':
        return None
        
    try:
        utc = row[0].strip()
        fix = row[1].strip()
        lat = float(row[2].strip())
        lon = float(row[3].strip())
        alt = float(row[4].strip()) if len(row) > 4 and row[4].strip() else 0.0
        sats_used = int(row[5].strip()) if len(row) > 5 and row[5].strip() else 0
        sats_view = int(row[6].strip()) if len(row) > 6 and row[6].strip() else 0
        hdop = float(row[7].strip()) if len(row) > 7 and row[7].strip() else 0.0
        speed = float(row[8].strip()) if len(row) > 8 and row[8].strip() else 0.0
        
        heading = 0.0
        if len(row) > 9 and row[9].strip():
            heading = float(row[9].strip())
            
        heading_cont = 0.0
        if len(row) > 10 and row[10].strip():
            heading_cont = float(row[10].strip())
            
        rtcm = int(row[11].strip()) if len(row) > 11 and row[11].strip() else 0
        
        return {
            'utc': utc, 'fix': fix, 'lat': lat, 'lon': lon, 'alt': alt,
            'sats_used': sats_used, 'sats_view': sats_view, 'hdop': hdop,
            'speed': speed, 'heading': heading, 'heading_cont': heading_cont,
            'rtcm': rtcm
        }
    except Exception:
        # Fallback split
        try:
            parts = [p.strip() for p in line_str.split(delimiter) if p.strip()]
            if len(parts) >= 4:
                return {
                    'utc': parts[0], 'fix': parts[1], 'lat': float(parts[2]), 'lon': float(parts[3]),
                    'alt': float(parts[4]) if len(parts) > 4 else 0.0,
                    'sats_used': int(parts[5]) if len(parts) > 5 else 0,
                    'sats_view': int(parts[6]) if len(parts) > 6 else 0,
                    'hdop': float(parts[7]) if len(parts) > 7 else 0.0,
                    'speed': float(parts[8]) if len(parts) > 8 else 0.0,
                    'heading': float(parts[9]) if len(parts) > 9 else 0.0,
                    'heading_cont': float(parts[10]) if len(parts) > 10 else 0.0,
                    'rtcm': int(parts[11]) if len(parts) > 11 else 0
                }
        except Exception:
            pass
            
    return None

def smooth_trajectory(points, window_size=5):
    """Applies a centered zero-phase moving average filter to smooth coordinates."""
    if len(points) < 3:
        return points
        
    smoothed = []
    half_w = window_size // 2
    
    for i in range(len(points)):
        start = max(0, i - half_w)
        end = min(len(points), i + half_w + 1)
        window = points[start:end]
        
        avg_x = sum(p[0] for p in window) / len(window)
        avg_y = sum(p[1] for p in window) / len(window)
        smoothed.append((avg_x, avg_y))
        
    return smoothed

def filter_outliers(records, max_speed_limit=5.0):
    """Filters out coordinates containing impossible instantaneous jumps (> 5 m/s)."""
    if len(records) < 2:
        return records
        
    clean_records = [records[0]]
    
    for i in range(1, len(records)):
        curr = records[i]
        prev = clean_records[-1]
        
        try:
            t_curr = float(curr['utc'])
            t_prev = float(prev['utc'])
            dt = abs(t_curr - t_prev)
        except ValueError:
            dt = 0.2  # Default to 5 Hz if UTC is non-numeric
            
        if dt <= 0:
            dt = 0.01
            
        dx = curr['utm_x'] - prev['utm_x']
        dy = curr['utm_y'] - prev['utm_y']
        dist = math.hypot(dx, dy)
        calc_speed = dist / dt
        
        if calc_speed > max_speed_limit:
            # Clamp coordinate update to hold previous position (spike removal)
            curr['utm_x'] = prev['utm_x']
            curr['utm_y'] = prev['utm_y']
            curr['lat'] = prev['lat']
            curr['lon'] = prev['lon']
            
        clean_records.append(curr)
        
    return clean_records

def main():
    if len(sys.argv) < 2:
        print("Usage: python process_rtk_log.py <input_log.csv> [output_log_smoothed.csv]")
        sys.exit(1)
        
    input_file = sys.argv[1]
    if not os.path.exists(input_file):
        print(f"Error: File '{input_file}' does not exist.")
        sys.exit(1)
        
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    else:
        base, ext = os.path.splitext(input_file)
        output_file = f"{base}_processed.csv"
        
    print(f"Reading input file: {input_file}")
    delim = detect_delimiter(input_file)
    
    raw_records = []
    with open(input_file, 'r', errors='ignore') as f:
        for line in f:
            record = parse_line_dict(line, delim)
            if record:
                ex, ny, zone = latlon_to_utm(record['lat'], record['lon'])
                record['utm_x'] = ex
                record['utm_y'] = ny
                record['utm_zone'] = zone
                raw_records.append(record)
                
    if not raw_records:
        print("Error: No valid RTK data rows parsed from the file.")
        sys.exit(1)
        
    print(f"Successfully parsed {len(raw_records)} records. Filtering spikes...")
    filtered_records = filter_outliers(raw_records)
    
    print("Smoothing coordinates via zero-phase moving average filter...")
    raw_coords = [(r['utm_x'], r['utm_y']) for r in filtered_records]
    smoothed_coords = smooth_trajectory(raw_coords, window_size=5)
    
    for idx, record in enumerate(filtered_records):
        record['smoothed_utm_x'] = smoothed_coords[idx][0]
        record['smoothed_utm_y'] = smoothed_coords[idx][1]
        
    print("Calculating path headings...")
    last_yaw = 0.0
    for i in range(len(filtered_records)):
        if i == 0:
            if len(filtered_records) > 1:
                dx = filtered_records[1]['smoothed_utm_x'] - filtered_records[0]['smoothed_utm_x']
                dy = filtered_records[1]['smoothed_utm_y'] - filtered_records[0]['smoothed_utm_y']
                last_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
            filtered_records[i]['calc_yaw_deg'] = last_yaw
            continue
            
        dx = filtered_records[i]['smoothed_utm_x'] - filtered_records[i-1]['smoothed_utm_x']
        dy = filtered_records[i]['smoothed_utm_y'] - filtered_records[i-1]['smoothed_utm_y']
        dist = math.hypot(dx, dy)
        
        # Only update heading if movement is significant to avoid noise integration
        if dist > 0.02:
            last_yaw = math.degrees(math.atan2(dy, dx)) % 360.0
        filtered_records[i]['calc_yaw_deg'] = last_yaw

    # Export processed path CSV
    headers = [
        'UTC', 'Fix_Status', 'Latitude', 'Longitude', 'Altitude',
        'Sats_Used', 'Sats_View', 'HDOP', 'Speed_Knots', 'Heading_Deg',
        'Continuous_Heading', 'RTCM_Bytes', 'UTM_X', 'UTM_Y', 'UTM_Zone',
        'Smoothed_UTM_X', 'Smoothed_UTM_Y', 'Calculated_Yaw_Deg'
    ]
    
    print(f"Writing output waypoint path: {output_file}")
    try:
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)
            for r in filtered_records:
                writer.writerow([
                    r['utc'], r['fix'], r['lat'], r['lon'], r['alt'],
                    r['sats_used'], r['sats_view'], r['hdop'], r['speed'], r['heading'],
                    r['heading_cont'], r['rtcm'], r['utm_x'], r['utm_y'], r['utm_zone'],
                    r['smoothed_utm_x'], r['smoothed_utm_y'], r['calc_yaw_deg']
                ])
        print("✅ Processing completed successfully!")
    except Exception as e:
        print(f"Error writing output file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
