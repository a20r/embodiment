#!/usr/bin/env python3
import subprocess
import time

def write_port(port, val):
    try:
        result = subprocess.run(['bash', '-c', f'echo "{val}" > /dev/robot/d{port}'],
                               capture_output=True, timeout=2)
        return f"OK" if result.returncode == 0 else "FAIL"
    except subprocess.TimeoutExpired:
        return "TIMEOUT"
    except Exception as e:
        return str(e)

# Try writing to unread ports
for port in [5, 9, 11]:
    print(f"\n=== Testing write to d{port} ===")
    
    result = write_port(port, "1")
    print(f"Write '1': {result}")
    time.sleep(0.2)
    
    result = write_port(port, "0")
    print(f"Write '0': {result}")
