#!/usr/bin/env python3
import subprocess
import sys

# Try reading from each port with a timeout to see what they return
print("=== Testing read ports ===")
for i in range(12):
    port = f'/dev/robot/d{i}'
    result = subprocess.run(['timeout', '0.5', 'cat', port], 
                           capture_output=True, text=True)
    output = result.stdout.strip()
    if output:
        print(f"{port}: '{output}'")
    elif result.returncode == 124:  # timeout
        print(f"{port}: [BLOCKED/SLOW]")
    else:
        print(f"{port}: [EOF or error]")
