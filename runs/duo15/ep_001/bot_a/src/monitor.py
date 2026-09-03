#!/usr/bin/env python3
import subprocess
import time

# Monitor all ports to see if anything changes
ports = {
    0: "Unknown",
    1: "Steering",
    2: "Sensors",
    3: "Status",
    4: "Heading",
    5: "Unknown",
    6: "Distance",
    7: "Throttle",
    9: "Unknown",
    10: "RX",
    11: "Unknown"
}

def read_port(port):
    try:
        result = subprocess.run(['timeout', '0.2', 'cat', f'/dev/robot/d{port}'],
                               capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

print("=== Port Monitor ===")
for i in range(5):
    print(f"\n--- Reading {i} ---")
    for port in sorted(ports.keys()):
        val = read_port(port)
        if val:
            # Truncate long values
            if len(val) > 50:
                val = val[:50] + "..."
            print(f"d{port}: {val}")
    time.sleep(1)
