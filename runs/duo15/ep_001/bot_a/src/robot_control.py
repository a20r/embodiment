#!/usr/bin/env python3
import subprocess
import time
import threading
import queue

class Robot:
    def __init__(self):
        self.message_queue = queue.Queue()
        self.running = True
        self.state = {
            'heading': 0,
            'distance': 0,
            'goal_reached': False,
            'tick': 0
        }
    
    def get_status(self):
        """Get robot status from d3"""
        result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d3'],
                               capture_output=True, text=True)
        try:
            parts = result.stdout.strip().split()
            tick = int(parts[0].split('=')[1])
            goal = int(parts[1].split('=')[1])
            here = int(parts[2].split('=')[1])
            return tick, goal, here
        except:
            return 0, 0, 0
    
    def get_heading(self):
        """Get heading from d4"""
        result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d4'],
                               capture_output=True, text=True)
        try:
            return float(result.stdout.strip())
        except:
            return 0.0
    
    def get_distance(self):
        """Get distance from d6"""
        result = subprocess.run(['timeout', '0.5', 'cat', '/dev/robot/d6'],
                               capture_output=True, text=True)
        try:
            return float(result.stdout.strip())
        except:
            return 0.0
    
    def set_throttle(self, value):
        """Set forward throttle on d7"""
        subprocess.run(['bash', '-c', f'echo "{value}" > /dev/robot/d7'],
                      capture_output=True, timeout=2)
    
    def set_steering(self, angle):
        """Set steering angle on d1"""
        subprocess.run(['bash', '-c', f'echo "{angle}" > /dev/robot/d1'],
                      capture_output=True, timeout=2)
    
    def send_message(self, msg):
        """Send message via transceiver d8"""
        subprocess.run(['bash', '-c', f'echo "{msg}" > /dev/robot/d8'],
                      capture_output=True, timeout=2)
        print(f"[TX] {msg}")
    
    def read_message(self):
        """Read message from transceiver d10 with timeout"""
        result = subprocess.run(['timeout', '0.2', 'cat', '/dev/robot/d10'],
                               capture_output=True, text=True)
        msg = result.stdout.strip()
        if msg:
            print(f"[RX] {msg}")
            return msg
        return None
    
    def listen_thread(self):
        """Background thread to listen for messages"""
        while self.running:
            msg = self.read_message()
            if msg:
                self.message_queue.put(msg)
            time.sleep(0.1)
    
    def main_loop(self):
        """Main control loop"""
        # Start listener thread
        listener = threading.Thread(target=self.listen_thread, daemon=True)
        listener.start()
        
        print("Robot initialized, starting handshake...")
        time.sleep(0.5)
        
        # Send initial hello
        self.send_message("HELLO")
        
        # Wait for response
        time.sleep(2)
        
        # Get initial position
        tick, goal, here = self.get_status()
        print(f"Initial status: tick={tick}, goal={goal}, here={here}")
        
        # Start moving forward
        print("Starting to move...")
        self.set_throttle(1)
        
        # Run for a bit
        for i in range(20):
            time.sleep(0.5)
            heading = self.get_heading()
            distance = self.get_distance()
            tick, goal, here = self.get_status()
            print(f"Tick {i}: heading={heading:.1f}°, dist={distance:.0f}, goal={goal}, here={here}")
            
            if here == 1:
                print("GOAL REACHED!")
                self.set_throttle(0)
                break
            
            # Check for messages
            try:
                while True:
                    msg = self.message_queue.get_nowait()
                    print(f"Got message: {msg}")
            except queue.Empty:
                pass
        
        self.set_throttle(0)
        self.running = False

if __name__ == '__main__':
    robot = Robot()
    robot.main_loop()
