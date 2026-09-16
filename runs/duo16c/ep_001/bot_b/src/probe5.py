import time, subprocess

def rd(p):
    r = subprocess.run(['timeout','1','cat',f'/dev/robot/{p}'], capture_output=True, text=True)
    return r.stdout.strip()

def stream(p, val, dur):
    end = time.time() + dur
    n = 0
    while time.time() < end:
        subprocess.run(['timeout','1','bash','-c',f"echo '{val}' > /dev/robot/{p}"], capture_output=True)
        n += 1
    return n

def lid_center():
    pts = rd('d2')
    ps = [tuple(map(float,t.split(','))) for t in pts.split(';') if t]
    return len(ps), round(sum(p[0] for p in ps)/len(ps),4) if ps else None

print('before', lid_center(), 'd4', rd('d4'))
n = stream('d1', '1.0', 3)
print('after d1=1.0 x3s', n, lid_center(), 'd4', rd('d4'))
n = stream('d7', '1.0', 3)
print('after d7=1.0 x3s', n, lid_center(), 'd4', rd('d4'))
