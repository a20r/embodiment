#!/usr/bin/env python3
"""
Robust robot explorer - handles goal detection and robot communication
"""
import subprocess
import time
import sys

def cmd(port, value):
    """Send command to port"""
    try:
        subprocess.run(f'echo "{value}" > /dev/robot/d{port}', 
                      shell=True, capture_output=True, timeout=1)
    except Exception as e:
        print(f"Error writing d{port}: {e}", file=sys.stderr)

def read(port, timeout_sec=0.2):
    """Read from port with timeout"""
    try:
        result = subprocess.run(f'timeout {timeout_sec} cat /dev/robot/d{port}',
                               shell=True, capture_output=True, text=True, timeout=timeout_sec+0.5)
        return result.stdout.strip()
    except:
        return ""

class RobotExplorer:
    def __init__(self):
        self.iteration = 0
        self.goal_found = False
        self.messages_received = []
        
    def update_state(self):
        """Get current robot state"""
        status = read(3)
        heading = read(4)
        distance = read(6)
        rx_msg = read(10)
        
        if 'here=1' in status:
            self.goal_found = True
            
        if rx_msg and len(rx_msg) > 0:
            self.messages_received.append((self.iteration, rx_msg))
            
        return {
            'status': status,
            'heading': heading,
            'distance': distance,
            'rx_msg': rx_msg
        }
    
    def search(self, max_iterations=2000):
        """Run search pattern"""
        print("[ROBOT] Starting exploration")
        
        cmd(7, 1)  # Start moving forward
        
        while self.iteration < max_iterations and not self.goal_found:
            # Update state
            state = self.update_state()
            
            # Change direction every 100 iterations
            if self.iteration % 100 == 0 and self.iteration > 0:
                angle = (self.iteration // 100 * 45) % 360
                cmd(1, angle)
            
            # Log periodically
            if self.iteration % 200 == 0:
                print(f"[{self.iteration}] {state['distance']}, heading={state['heading']}")
            
            # Check for goal
            if self.goal_found:
                print(f"[GOAL] Found at iteration {self.iteration}!")
                break
            
            self.iteration += 1
            time.sleep(0.15)
        
        cmd(7, 0)  # Stop
        
        print(f"[DONE] Completed {self.iteration} iterations")
        if self.messages_received:
            print(f"[MSGS] Received {len(self.messages_received)} messages")
            for it, msg in self.messages_received[:5]:
                print(f"  [{it}] {msg}")

if __name__ == '__main__':
    explorer = RobotExplorer()
    explorer.search()
