#!/usr/bin/env python3
import subprocess
import sys
import time

# Try writing to write-only ports
write_ports = {
    1: "Forward command?",
    7: "Unknown",
    8: "Transceiver TX"
}

for port, desc in write_ports.items():
    port_path = f'/dev/robot/d{port}'
    print(f"\n=== Testing d{port} ({desc}) ===")
    
    # Try writing a simple command
    result = subprocess.run(['timeout', '2', 'bash', '-c', 
                           f'echo "1" > {port_path}'],
                           capture_output=True, text=True, timeout=3)
    
    if result.returncode == 0:
        print(f"Write successful (no error)")
        # Check if anything changed
        time.sleep(0.2)
        status = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d3'],
                               capture_output=True, text=True)
        print(f"Status: {status.stdout.strip()}")
    elif result.returncode == 124:
        print(f"Write blocked (timeout)")
    else:
        print(f"Write failed: {result.stderr}")
