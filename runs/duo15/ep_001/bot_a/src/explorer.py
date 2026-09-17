#!/usr/bin/env python3
import subprocess
import time
import threading
import queue
import math

class RobotExplorer:
    def __init__(self):
        self.running = True
        self.messages = queue.Queue()
        self.state = {
            'heading': 0,
            'distance': 0,
            'at_goal': False,
            'tick': 0,
            'start_time': time.time()
        }
        self.search_pattern = 0  # 0=forward, 1=turn left, 2=turn right
        self.last_broadcast = 0
        
    def read_port(self, port):
        """Read from a port safely"""
        try:
            result = subprocess.run(['timeout', '0.5', 'cat', f'/dev/robot/d{port}'],
                                   capture_output=True, text=True, timeout=1)
            return result.stdout.strip()
        except:
            return ""
    
    def write_port(self, port, value):
        """Write to a port safely"""
        try:
            subprocess.run(['bash', '-c', f'echo "{value}" > /dev/robot/d{port}'],
                          capture_output=True, timeout=2)
        except:
            pass
    
    def get_status(self):
        """Parse d3 status"""
        data = self.read_port(3)
        try:
            parts = data.split()
            tick = int(parts[0].split('=')[1])
            goal = int(parts[1].split('=')[1])
            here = int(parts[2].split('=')[1])
            return tick, goal, here
        except:
            return 0, 0, 0
    
    def get_heading(self):
        """Get heading from d4"""
        try:
            return float(self.read_port(4))
        except:
            return 0.0
    
    def get_distance(self):
        """Get distance from d6"""
        try:
            return float(self.read_port(6))
        except:
            return 0.0
    
    def broadcast(self, msg):
        """Send message to other robot"""
        self.write_port(8, msg)
        print(f"[BROADCAST] {msg}")
    
    def listen_thread(self):
        """Listen for incoming messages"""
        while self.running:
            data = self.read_port(10)
            if data and len(data) > 0:
                print(f"[RECEIVED] {data}")
                self.messages.put(data)
            time.sleep(0.2)
    
    def move_forward(self, duration=1.0):
        """Move forward for a duration"""
        self.write_port(7, 1)
        time.sleep(duration)
        self.write_port(7, 0)
    
    def turn(self, angle):
        """Set steering angle"""
        self.write_port(1, angle)
    
    def stop(self):
        """Stop moving"""
        self.write_port(7, 0)
        self.turn(0)
    
    def search(self):
        """Main search loop"""
        # Start listener
        listener = threading.Thread(target=self.listen_thread, daemon=True)
        listener.start()
        
        print("Starting exploration...")
        self.broadcast("HELLO_IM_ALIVE")
        
        # Simple search: move forward, if stuck turn
        move_counter = 0
        turn_direction = 1  # 1 for left, -1 for right
        
        while self.running:
            tick, goal, here = self.get_status()
            heading = self.get_heading()
            distance = self.get_distance()
            
            elapsed = time.time() - self.state['start_time']
            print(f"[{elapsed:.1f}s] tick={tick}, here={here}, heading={heading:.1f}°, dist={distance:.0f}")
            
            if here == 1:
                print("*** GOAL REACHED! ***")
                self.stop()
                self.broadcast("GOAL_REACHED")
                self.state['at_goal'] = True
                break
            
            # Broadcast status every 5 seconds
            if time.time() - self.last_broadcast > 5:
                self.broadcast(f"POS:h={heading:.0f},d={distance:.0f}")
                self.last_broadcast = time.time()
            
            # Simple strategy: move forward, occasionally turn to search
            move_counter += 1
            if move_counter % 20 == 0:
                # Every 20 iterations, try turning
                self.turn(turn_direction * 15)
                time.sleep(0.3)
                self.turn(0)
            
            # Move
            self.write_port(7, 1)  # Keep moving
            time.sleep(0.5)
            
            if elapsed > 60:  # Safety timeout
                print("Timeout, stopping")
                break
        
        self.stop()
        self.running = False

if __name__ == '__main__':
    explorer = RobotExplorer()
    explorer.search()
