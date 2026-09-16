import time, subprocess

def rd(p, tries=5):
    for _ in range(tries):
        r = subprocess.run(['timeout','1','cat',f'/dev/robot/{p}'], capture_output=True, text=True)
        if r.stdout.strip():
            return r.stdout.strip()
        time.sleep(0.05)
    return ''

def stream2(v1, v7, dur):
    end = time.time() + dur
    n=0
    while time.time() < end:
        subprocess.run(['timeout','1','bash','-c',f"echo '{v1}' > /dev/robot/d1"], capture_output=True)
        subprocess.run(['timeout','1','bash','-c',f"echo '{v7}' > /dev/robot/d7"], capture_output=True)
        n+=1
    return n

def stats():
    pts=[tuple(map(float,t.split(','))) for t in rd('d2').split(';') if t]
    if not pts: return None
    xs=[p[0] for p in pts]
    front=[p for p in pts if -0.2<p[1]<0.2 and p[0]>0]
    return len(pts), round(min(xs),3), round(sum(xs)/len(xs),4), len(front)

print('before', stats(), 'd4', rd('d4'))
n = stream2('1.0','1.0', 3)
print('both fwd 3s:', n, stats(), 'd4', rd('d4'), 'd0', rd('d0'))
n = stream2('-1.0','1.0', 3)
print('spin 3s:', n, stats(), 'd4', rd('d4'), 'd0', rd('d0'))
