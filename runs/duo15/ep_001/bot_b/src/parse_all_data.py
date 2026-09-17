#!/usr/bin/env python3
import subprocess
import time

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.2 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.5
        )
        if result.stdout:
            return result.stdout.strip()
    except:
        pass
    return None

# Collect all data
print("Full robot state:")
print()

data = {}
for port in range(12):
    val = safe_read(port)
    if val:
        data[f'd{port}'] = val
        print(f"d{port:2d}: {val}")

print()
print("Interpretation:")
print(f"- d3 (status): {data.get('d3', 'N/A')} - Goal={data.get('d3', '').split('goal=')[1][:1] if 'goal=' in data.get('d3', '') else '?'}")
print(f"- d4 (bearing): {data.get('d4', 'N/A')}° - Robot heading")
print(f"- d9: {data.get('d9', 'N/A')} - Position X?")
print(f"- d11: {data.get('d11', 'N/A')} - Position Y?")

# Maybe position is encoded in d9 and d11?
try:
    d9_val = float(data.get('d9', 0))
    d11_val = float(data.get('d11', 0))
    print(f"\nEstimated position: ({d9_val:.1f}, {d11_val:.3f})")
except:
    pass

