#!/usr/bin/env python3
import time
import sys
import threading

class Robot:
    def __init__(self):
        self.running = True
        self.last_status = {}
        
    def read_port(self, port_num, timeout=1):
        """Try to read from a device port"""
        try:
            port_path = f"/dev/robot/d{port_num}"
            with open(port_path, 'r') as f:
                line = f.readline()
                return line.strip()
        except:
            pass
        return None

    def write_port(self, port_num, message):
        """Write to a device port"""
        try:
            port_path = f"/dev/robot/d{port_num}"
            with open(port_path, 'w') as f:
                f.write(message + '\n')
                f.flush()
            return True
        except Exception as e:
            print(f"Error writing to d{port_num}: {e}", file=sys.stderr)
            return False
    
    def get_status(self):
        """Get current status"""
        status_str = self.read_port(3)
        if status_str:
            status = {}
            for part in status_str.split():
                if '=' in part:
                    k, v = part.split('=')
                    try:
                        status[k] = int(v)
                    except:
                        status[k] = v
            return status
        return {}
    
    def send_message(self, msg):
        """Send message to other robot"""
        return self.write_port(8, msg)
    
    def receive_message(self, timeout=0.5):
        """Receive message from other robot"""
        return self.read_port(10, timeout=timeout)
    
    def move_forward(self, speed=1):
        """Move forward"""
        return self.write_port(1, str(speed))
    
    def turn(self, angle_offset):
        """Turn - try different ports"""
        # Maybe d6 or d7 controls turning?
        return self.write_port(6, str(angle_offset))
    
    def get_bearing(self):
        """Get current bearing in degrees"""
        bearing_str = self.read_port(4)
        if bearing_str:
            try:
                return float(bearing_str)
            except:
                pass
        return None
    
    def get_sensors(self):
        """Get sensor readings (possibly LIDAR or distance sensors)"""
        sensors_str = self.read_port(2)
        if sensors_str:
            try:
                return [float(x) for x in sensors_str.split(',')]
            except:
                pass
        return []

robot = Robot()

# Initial status check
print("=== Robot Control System ===")
status = robot.get_status()
print(f"Status: {status}")
bearing = robot.get_bearing()
print(f"Bearing: {bearing}")
sensors = robot.get_sensors()
print(f"Sensors: {sensors}")

# Try announcing ourselves
print("\nSending announcement...")
robot.send_message("ROBOT_1_ONLINE")

# Listen for response
print("Listening for other robot...")
for _ in range(5):
    msg = robot.receive_message(timeout=0.5)
    if msg:
        print(f"Received: {msg}")
    time.sleep(0.2)

