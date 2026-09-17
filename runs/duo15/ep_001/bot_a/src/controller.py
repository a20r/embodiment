#!/usr/bin/env python3
import sys
import os
import time
import select

def read_with_timeout(fd, timeout=2.0):
    """Read from a file descriptor with timeout"""
    ready = select.select([fd], [], [], timeout)
    if ready[0]:
        return os.read(fd, 1024).decode('utf-8').strip()
    return None

def write_message(fd, msg):
    """Write a message to a file descriptor"""
    os.write(fd, (msg + '\n').encode('utf-8'))

# Open transceiver ports
tx_fd = os.open('/dev/robot/d8', os.O_WRONLY)
rx_fd = os.open('/dev/robot/d10', os.O_RDONLY | os.O_NONBLOCK)

print("Connected to transceiver")

# Send initial handshake
print("Sending handshake...")
write_message(tx_fd, "HELLO")

# Try to receive response
print("Listening for response...")
time.sleep(0.5)
response = read_with_timeout(rx_fd, timeout=3.0)
print(f"Response: {response}")

# Clean up
os.close(tx_fd)
os.close(rx_fd)
