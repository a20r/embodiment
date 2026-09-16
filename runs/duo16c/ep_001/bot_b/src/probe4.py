import time, subprocess

def rd(p):
    r = subprocess.run(['timeout','1','cat',f'/dev/robot/{p}'], capture_output=True, text=True)
    return r.stdout.strip()

def wr(p, s):
    subprocess.run(['timeout','1','bash','-c',f"echo '{s}' > /dev/robot/{p}"], capture_output=True)

def lid_center():
    pts = rd('d2')
    ps = [tuple(map(float,t.split(','))) for t in pts.split(';') if t]
    return len(ps), (sum(p[0] for p in ps)/len(ps) if ps else None)

print('lidar', lid_center(), 'd4', rd('d4'))
for cmd in ['1.0', '1 0', '0,1', '1.0 0.0', 'f 1', 'w 1']:
    wr('d1', cmd); wr('d7', cmd)
    time.sleep(1.5)
    print(f'cmd={cmd!r} lidar', lid_center(), 'd4', rd('d4'), 'd3', rd('d3'))
wr('d1','0'); wr('d7','0')
