import math, time, subprocess, os

def read_line(p, timeout=1.0):
    r = subprocess.run(['timeout',str(timeout),'cat',f'/dev/robot/{p}'], capture_output=True, text=True)
    return r.stdout.strip()

def lidar_pts():
    for _ in range(5):
        s = read_line('d2')
        if s:
            return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

def descriptor():
    pts = lidar_pts()
    if not pts: return None
    # azimuth-range grid 16x8
    grid = [0.0]*128
    for x,y,z in pts:
        az = math.atan2(y,x)
        r = math.hypot(x,y)
        b = int((az+math.pi)/(2*math.pi)*16) % 16
        rb = min(7, int(r/0.4))
        grid[b*8+rb] += 1
    tot = sum(grid) or 1
    return [g/tot for g in grid]

def diff(a,b):
    return sum(abs(x-y) for x,y in zip(a,b))/2  # L1/2 distance

def stream(v1, v7, dur, period=0.02):
    end = time.time()+dur
    n=0
    while time.time()<end:
        for p,v in (('d1',v1),('d7',v7)):
            fd = os.open(f'/dev/robot/{p}', os.O_WRONLY|os.O_NONBLOCK)
            try: os.write(fd, (v+'\n').encode())
            except Exception: pass
            os.close(fd)
        time.sleep(period); n+=1
    return n
