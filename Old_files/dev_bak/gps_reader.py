from serial import Serial
from pyubx2 import UBXReader

stream = Serial('/dev/ttyACM0', 115200, timeout=3)
ubr = UBXReader(stream, protfilter=2)

print("ZED-F9P GPS Reader")
print("-" * 40)

while True:
    raw, parsed = ubr.read()
    if parsed and parsed.identity == "NAV-PVT":
        print(f"Lat: {parsed.lat:.7f}, Lon: {parsed.lon:.7f}")
        print(f"Fix: {parsed.fixType}, Sats: {parsed.numSV}, hAcc: {parsed.hAcc/10:.1f}mm")
        print(f"RTK: {'Float' if parsed.carrSoln==1 else 'Fixed' if parsed.carrSoln==2 else 'None'}")
        print("-" * 40)
