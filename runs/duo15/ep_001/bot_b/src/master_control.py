#!/usr/bin/env python3
"""
Master control script - runs multiple strategies to find goal and other robot
"""
import subprocess
import time
import threading

def safe_read(port_num):
    try:
        result = subprocess.run(
            f"timeout 0.1 cat /dev/robot/d{port_num}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=0.3
        )
        return result.stdout.strip() if result.stdout else None
    except:
        return None

def safe_write(port_num, message):
    try:
        subprocess.run(
            f"echo '{message}' > /dev/robot/d{port_num}",
            shell=True,
            timeout=0.3,
            capture_output=True
        )
    except:
        pass

class MasterRobot:
    def __init__(self):
        self.running = True
        self.goal_found = False
        self.message_count = 0
        
    def broadcaster(self):
        """Continuously broadcast to other robot"""
        while self.running:
            self.message_count += 1
            d9 = safe_read(9)
            d11 = safe_read(11)
            msg = f"R1:{d9},{d11},{self.message_count}"
            safe_write(8, msg)
            time.sleep(0.5)
    
    def listener(self):
        """Listen for responses"""
        while self.running:
            msg = safe_read(10)
            if msg and msg.strip():
                print(f"[RECEIVED] {msg}")
            time.sleep(0.2)
    
    def goal_monitor(self):
        """Monitor for goal trigger"""
        while self.running:
            status = safe_read(3)
            if status and 'goal=1' in status:
                print(f"\n*** GOAL FOUND! ***")
                print(f"Status: {status}")
                self.goal_found = True
                self.running = False
                break
            time.sleep(0.5)
    
    def movement_strategy(self):
        """Execute movement strategy"""
        # Start by moving forward
        safe_write(1, "1")
        time.sleep(5)
        
        # Zigzag pattern
        for i in range(10):
            if not self.running:
                break
                
            # Turn
            angle = (i * 45) % 360
            safe_write(6, str(angle))
            time.sleep(3)
            
            # Check position
            d9 = safe_read(9)
            print(f"[{i}] Position d9={d9}, angle={angle}")
        
        # Move backward
        safe_write(1, "-1")
        time.sleep(10)
        
        # Stop
        safe_write(1, "0")
    
    def run(self):
        """Start all threads"""
        print("Starting master robot control...")
        
        t1 = threading.Thread(target=self.broadcaster, daemon=True)
        t2 = threading.Thread(target=self.listener, daemon=True)
        t3 = threading.Thread(target=self.goal_monitor, daemon=True)
        
        t1.start()
        t2.start()
        t3.start()
        
        # Movement happens on main thread
        try:
            self.movement_strategy()
        except Exception as e:
            print(f"Error: {e}")
        
        # Wait for threads
        t1.join(timeout=5)
        t2.join(timeout=5)
        t3.join(timeout=5)
        
        print("Master control finished")

if __name__ == "__main__":
    robot = MasterRobot()
    robot.run()
