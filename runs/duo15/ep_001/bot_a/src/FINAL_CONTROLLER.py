#!/usr/bin/env python3
"""
FINAL ROBOT CONTROLLER
This is the main control program for autonomous navigation to goal.

Status: SEARCHING FOR GOAL
- Robot can move forward/backward
- Robot can steer
- No goal location found yet
- No contact with other robot

This controller will:
1. Continuously search for goal
2. Listen for other robot messages
3. Execute coordinated navigation if other robot detected
"""

import subprocess
import time
import threading

class RobotController:
    def __init__(self):
        self.running = True
        self.goal_found = False
        self.other_robot_found = False
        self.goal_position = None
        
    def cmd(self, port, value):
        """Write to robot port"""
        try:
            subprocess.run(f'echo "{value}" > /dev/robot/d{port}', 
                          shell=True, capture_output=True, timeout=1)
        except:
            pass
    
    def read(self, port, timeout_sec=0.2):
        """Read from robot port"""
        try:
            r = subprocess.run(f'timeout {timeout_sec} cat /dev/robot/d{port}',
                             shell=True, capture_output=True, text=True, timeout=timeout_sec+0.5)
            return r.stdout.strip()
        except:
            return ""
    
    def move_forward(self):
        """Move forward"""
        self.cmd(7, 1)
    
    def stop(self):
        """Stop moving"""
        self.cmd(7, 0)
    
    def turn(self, angle):
        """Set steering angle"""
        self.cmd(1, angle)
    
    def broadcast(self, msg):
        """Send message to other robot"""
        self.cmd(8, msg)
    
    def get_status(self):
        """Get current status"""
        status = self.read(3)
        heading = self.read(4)
        distance = self.read(6)
        message = self.read(10)
        
        if 'here=1' in status:
            self.goal_found = True
        
        return {
            'status': status,
            'heading': heading,
            'distance': distance,
            'message': message
        }
    
    def search(self, duration_seconds=120):
        """Run search for specified duration"""
        start = time.time()
        iteration = 0
        
        self.move_forward()
        
        while time.time() - start < duration_seconds and not self.goal_found:
            state = self.get_status()
            
            # Change direction periodically
            if iteration % 100 == 0 and iteration > 0:
                angle = (iteration // 100 * 45) % 360
                self.turn(angle)
            
            # Broadcast periodically
            if iteration % 50 == 0:
                self.broadcast(f"ROBOT_1_SEARCHING_{iteration}")
            
            # Check results
            if self.goal_found:
                print(f"[GOAL FOUND] at iteration {iteration}")
                break
            
            if state['message']:
                print(f"[MESSAGE] {state['message']}")
                self.other_robot_found = True
            
            iteration += 1
            time.sleep(0.1)
        
        self.stop()
        
        return {
            'iterations': iteration,
            'goal_found': self.goal_found,
            'message_received': self.other_robot_found,
            'duration': time.time() - start
        }

if __name__ == '__main__':
    controller = RobotController()
    result = controller.search(duration_seconds=60)
    print(f"Search complete: {result}")
