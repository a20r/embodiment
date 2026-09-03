#!/usr/bin/env python3
import os
import sys

# Try to write to various ports to understand what they control
ports = [f'/dev/robot/d{i}' for i in range(12)]

for port in ports:
    try:
        fd = os.open(port, os.O_WRONLY)
        os.write(fd, b"TEST\n")
        print(f"{port}: WRITE OK")
        os.close(fd)
    except Exception as e:
        print(f"{port}: {e}")
