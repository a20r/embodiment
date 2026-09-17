import time, math, json, threading
from robot import Robot

logf = open('/bot/src/explore2.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

r = Robot()
r.stop(); time.sleep(0.3)

TICKS = 544.0
x=y=0.0
h = r.heading() or 0.0
le, re = r.enc()
trail = [(x,y)]

def beam(lid, i):
    v = lid[i % 16]
    if v is None or v < 0: 
        # use neighbors
        a = lid[(i-1)%16]; b = lid[(i+1)%16]
        cands = [z for z in (a,b) if z and z>0]
        v = min(cands) if cands else 0.5
    return v

def zones(lid):
    front = min(beam(lid,15), beam(lid,0), beam(lid,1))
    right = min(beam(lid,3), beam(lid,4), beam(lid,5))
    left  = min(beam(lid,11), beam(lid,12), beam(lid,13))
    return front, right, left

def update_pose():
    global x,y,h,le,re
    l2, r2 = r.enc()
    if l2 is None: return
    h2 = r.heading()
    if h2 is None: return
    dl = (l2-le)/TICKS; dr = (r2-re)/TICKS
    le, re = l2, r2
    h = h2
    d = (dl+dr)/2
    rad = math.radians(h)
    x += d*math.sin(rad); y += d*math.cos(rad)

def drive_step(dist=0.22, speed=20, timeout=8):
    """drive forward ~dist with front safety; returns True if completed"""
    global trail
    l0, r0 = r.enc()
    if l0 is None: return False
    t0 = time.time()
    target_ticks = dist*TICKS
    while time.time()-t0 < timeout:
        update_pose()
        l1, r1 = r.enc()
        if l1 is None: continue
        prog = ((l1-l0)+(r1-r0))/2.0
        if prog >= target_ticks: 
            trail.append((x,y))
            return True
        lid = r.lidar()
        if lid:
            front, right, left = zones(lid)
            if front < 0.17:
                r.stop(); log('front blocked at', f'{x:.2f},{y:.2f}'); 
                trail.append((x,y))
                return False
            base = speed if front > 0.3 else 10
            # wall follow correction: keep left wall ~0.22
            err = left - 0.22   # >0: too far from left wall -> steer left
            corr = max(-6, min(6, err*25))
            r.wheels(base - corr, base + corr)  # hmm sign check below
        else:
            r.wheels(speed, speed)
        time.sleep(0.04)
    r.stop(); return False

def turn(deg, speed=25, timeout=10):
    """rotate in place by deg (+=CW) using compass feedback"""
    t0 = time.time()
    h0 = r.heading() or h
    target = (h0 + deg) % 360
    while time.time()-t0 < timeout:
        hc = r.heading()
        if hc is None: continue
        err = (target - hc + 180) % 360 - 180
        if abs(err) <= 5: break
        v = max(10, min(30, abs(err)))
        if err > 0: r.wheels(v, -v)
        else: r.wheels(-v, v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.15)

def radio():
    while True:
        r.send(f'A PING {x:.1f} {y:.1f} {h:.0f}')
        t0 = time.time()
        while time.time()-t0 < 7:
            v = r.recv(0.4)
            if v: log('RADIO RX:', v)
            time.sleep(0.2)

threading.Thread(target=radio, daemon=True).start()

log('=== explore2 start ===')
st = r.status()
log('status', st, 'h', h)
try:
    for step in range(400):
        st = r.status()
        if st and (st[1] or st[2]):
            log('FLAG!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            break
        lid = r.lidar()
        if not lid: continue
        front, right, left = zones(lid)
        # left-hand rule
        if left > 0.35 and front > 0.25:
            log(f'{step}: turn LEFT (L={left:.2f} F={front:.2f}) at {x:.2f},{y:.2f}')
            turn(-88)
        elif front > 0.32:
            done = drive_step()
            if not done: log(f'{step}: step incomplete F={front:.2f} at {x:.2f},{y:.2f}')
        else:
            log(f'{step}: turn RIGHT (L={left:.2f} F={front:.2f} R={right:.2f}) at {x:.2f},{y:.2f}')
            turn(88)
        if step % 20 == 0:
            log(f'step {step} pose {x:.2f},{y:.2f} h={h:.0f} batt={r.battery()}')
            json.dump(trail, open('/bot/src/trail.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    json.dump(trail, open('/bot/src/trail.json','w'))
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
