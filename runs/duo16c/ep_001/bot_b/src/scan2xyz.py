import math
def scan(fn='/tmp/scan0.txt'):
    pts=[]
    for t in open(fn).read().split(';'):
        t=t.strip()
        if t:
            x,y,z=map(float,t.split(','))
            pts.append((x,y,z))
    return pts
def summarize(pts):
    import collections
    # 2D range profile by azimuth (16 bins)
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.atan2(y,x)
        b=int((a+math.pi)/(2*math.pi)*16)%16
        az[b].append(r)
    for b in range(16):
        rs=sorted(az[b])
        if rs:
            print(f'az bin {b:2d} [{-180+b*22.5:6.1f}deg]: n={len(rs):4d} min={rs[0]:.2f} p25={rs[len(rs)//4]:.2f} med={rs[len(rs)//2]:.2f} max={rs[-1]:.2f}')
        else:
            print(f'az bin {b:2d}: EMPTY')
if __name__=='__main__':
    summarize(scan())
