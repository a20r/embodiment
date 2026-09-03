#!/usr/bin/env python3
"""
Main robot control loop.
Strategies:
1. Continuously announce presence
2. Move systematically to search for other robot
3. When in communication range, coordinate to reach goal
"""
import subprocess
import time
import threading
import json

class Robot:
    def __init__(self):
        self.running = True
        self.last_recv = None
        self.tick = 0
        
    def safe_read(self, port_num):
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

    def safe_write(self, port_num, message):
        try:
            subprocess.run(
                f"echo '{message}' > /dev/robot/d{port_num}",
                shell=True,
                timeout=0.5,
                capture_output=True
            )
        except:
            pass
    
    def get_status(self):
        status_str = self.safe_read(3)
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
    
    def broadcast(self):
        """Send status to other robot"""
        status = self.get_status()
        msg = f"ROBOT1:tick={status.get('tick',0)}"
        self.safe_write(8, msg)
    
    def listen(self):
        """Try to receive message"""
        msg = self.safe_read(10)
        if msg:
            self.last_recv = msg
            print(f"[RECV] {msg}")
            return msg
        return None
    
    def move_forward(self):
        self.safe_write(1, "1")
    
    def stop(self):
        self.safe_write(1, "0")
    
    def turn(self, angle):
        self.safe_write(6, str(angle))
    
    def run(self, duration=60):
        """Run robot for specified duration"""
        print(f"Starting robot for {duration} seconds...")
        start_time = time.time()
        
        # Start moving
        self.move_forward()
        
        step = 0
        while time.time() - start_time < duration:
            # Broadcast every 0.5 seconds
            if step % 3 == 0:
                self.broadcast()
                print(f"[{step}] Broadcasting...")
            
            # Listen
            self.listen()
            
            # Check for goal
            status = self.get_status()
            if status.get('goal') == 1:
                print(f"*** GOAL REACHED! ***")
                self.stop()
                break
            
            # Change direction occasionally
            if step % 20 == 10:
                angle = (step // 20) * 45
                self.turn(angle)
                print(f"[{step}] Turning to {angle}°")
            
            time.sleep(0.5)
            step += 1
        
        self.stop()
        print("Robot run complete")

# Run the robot
robot = Robot()
robot.run(duration=90)
