#!/usr/bin/env python3
import os
import sys
import time

# Try to read from various ports to understand what they do
ports = [f'/dev/robot/d{i}' for i in range(12)]

for port in ports:
    try:
        # Try opening in read mode with timeout
        fd = os.open(port, os.O_RDONLY | os.O_NONBLOCK)
        try:
            data = os.read(fd, 256).decode('utf-8', errors='ignore').strip()
            print(f"{port}: {data if data else '(empty or blocked)'}")
        except:
            print(f"{port}: (no data or error)")
        os.close(fd)
    except Exception as e:
        print(f"{port}: {e}")
