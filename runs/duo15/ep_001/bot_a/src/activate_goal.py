import subprocess, time
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)
def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()

print("Trying activation sequences...")

# Try 1: Set different ports in sequence
print("Seq1: Setting ports...")
cmd(5, "1")
cmd(6, "1")
cmd(9, "1")
time.sleep(1)
print(f"d3: {read(3)}")

# Try 2: Send specific transceiver format
print("\nSeq2: Sending formatted message...")
cmd(8, "goal:start")
time.sleep(0.5)
print(f"RX: {read(10)}")

# Try 3: Reset and try
print("\nSeq3: Reset sequence...")
cmd(7, 0)
cmd(1, 0)
time.sleep(0.5)
cmd(7, 1)
for i in range(10):
    if 'here=1' in read(3):
        print(f"GOAL at step {i}!")
        break
    time.sleep(0.2)

print("Done")
