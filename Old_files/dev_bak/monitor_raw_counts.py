import time
import os

def read_raw(path):
    try:
        with open(path, 'r') as f:
            return int(f.read().strip())
    except Exception as e:
        return f"ERROR ({e})"

def main():
    p1 = "/sys/bus/counter/devices/counter1/count0/count"
    p2 = "/sys/bus/counter/devices/counter2/count0/count"
    
    print("Polling raw eQEP counts. Please rotate the wheels...")
    print(f"{'Time (s)':<10} | {'Counter 1 (Left)':<20} | {'Counter 2 (Right)':<20}")
    print("-" * 60)
    
    t0 = time.monotonic()
    for i in range(20):
        t = time.monotonic() - t0
        c1 = read_raw(p1)
        c2 = read_raw(p2)
        print(f"{t:<10.2f} | {c1:<20} | {c2:<20}")
        time.sleep(0.25)

if __name__ == "__main__":
    main()
