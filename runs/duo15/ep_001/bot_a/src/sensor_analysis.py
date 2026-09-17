import subprocess, time

def read(p): return subprocess.run(f'timeout 0.2 cat /dev/robot/d{p}', shell=True, capture_output=True, text=True, timeout=1).stdout.strip()
def cmd(p,v): subprocess.run(f'echo "{v}" > /dev/robot/d{p}', shell=True, capture_output=True, timeout=1)

print("Analyzing sensor readings while moving...")
cmd(7, 1)

prev_max = 0
max_sensor_ever = 0

for i in range(50):
    sensor_str = read(2)
    if not sensor_str: continue
    
    sensors = [float(x) for x in sensor_str.split(',')]
    max_val = max(sensors)
    max_idx = sensors.index(max_val)
    
    if max_val > max_sensor_ever:
        max_sensor_ever = max_val
        print(f"[{i}] NEW MAX: {max_val:.3f} at sensor {max_idx}")
    
    if i % 10 == 0:
        print(f"[{i}] max_sensor={max_val:.3f}, min={min(sensors):.3f}")
    
    time.sleep(0.3)

cmd(7, 0)
print(f"Max sensor value seen: {max_sensor_ever:.3f}")
