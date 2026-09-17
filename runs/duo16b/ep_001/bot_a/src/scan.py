import os, select, time, math, json

D='/dev/robot/'
def read(p, timeout=0.25):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out = os.read(fd, 2000000).decode().strip()
        except Exception: out=''
    os.close(fd)
    return out

def w(p, msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p, os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception as e: print("werr",p,e)

def lidar_pts():
    s = read('d2')
    pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            a,b,c = p.split(',')
            pts.append((float(a),float(b),float(c)))
        except: pass
    return pts

# pan-scan: rotate in steps, collect (heading, min positive range in cone)
def panscan(step_deg=15.0, full=360.0):
    start_h = float(read('d4'))
    results=[]
    n = int(full/step_deg)
    for i in range(n):
        pts = lidar_pts()
        rs = [r for r,e,a in pts if r > 0]
        mn = min(rs) if rs else 99.0
        mean = sum(rs)/len(rs) if rs else 99.0
        h = float(read('d4'))
        results.append((h, mn, mean))
        # rotate step
        target = (h + step_deg) % 360
        t0=time.time()
        while time.time()-t0 < 1.5:
            cur = float(read('d4') or h)
            err = ((target - cur + 180) % 360) - 180
            if abs(err) < 3: break
            spd = max(-60, min(60, err*3))
            w('d1', spd); w('d7', -spd)
            time.sleep(0.08)
        w('d1',0); w('d7',0)
        time.sleep(0.2)
    return results

if __name__ == '__main__':
    res = panscan()
    for h,mn,mean in res: print(f"h={h:6.1f} min={mn:.3f} mean={mean:.3f}")
    with open('/memory/scan_last.json','w') as f: json.dump(res,f)
