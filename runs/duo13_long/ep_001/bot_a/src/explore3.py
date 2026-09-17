import time, math, signal, sys, json, threading
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/explore3.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0
h = r.heading() or 0.0
le, re = r.enc()
r.stop(); time.sleep(0.3)

CELL = 0.35
visited = set()
def ckey(px,py,ph):
    return (int(round(px/CELL)), int(round(py/CELL)))
def update_pose():
    global x,y,h,le,re
    l2,r2 = r.enc()
    if l2 is None: return
    h2 = r.heading()
    if h2 is None: return
    dl=(l2-le)/TICKS; dr=(r2-re)/TICKS
    le,re=l2,r2; h=h2
    d=(dl+dr)/2
    rad=math.radians(h)
    x+=d*math.sin(rad); y+=d*math.cos(rad)

def clean(lid):
    out=[]
    for i,v in enumerate(lid):
        if v is None or v<0:
            a=lid[(i-1)%16]; b=lid[(i+1)%16]
            c=[z for z in (a,b) if z and z>0]
            v=min(c) if c else 0.4
        out.append(v)
    return out

def rotto(tg, timeout=8, tol=5):
    t0=time.time()
    while time.time()-t0<timeout:
        hc=r.heading()
        if hc is None: continue
        e=(tg-hc+180)%360-180
        if abs(e)<=tol: break
        v=max(10,min(28,abs(e)*1.1))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.12)

def drive(dist_max, timeout=12):
    """drive forward until front blocked (<0.22) or dist_max reached; returns dist done"""
    global x,y,h
    l0,r0 = r.enc()
    if l0 is None: return 0
    t0=time.time()
    done=0.0
    while time.time()-t0<timeout:
        update_pose()
        l1,r1=r.enc()
        if l1 is None: continue
        prog=((l1-l0)+(r1-r0))/2.0
        if prog >= dist_max*TICKS: break
        lid=r.lidar()
        if lid:
            c=clean(lid)
            f=min(c[15],c[0],c[1])
            if f<0.20: break
            sp = 22 if f>0.4 else 12
            # center in corridor: use beams 3-4 (right diag/side) and 12-13
            err = (min(c[12],c[13]) - min(c[3],c[4]))
            corr = max(-7,min(7, err*20))
            r.wheels(sp-corr, sp+corr)
        else:
            r.wheels(15,15)
        time.sleep(0.03)
    r.stop(); time.sleep(0.1)
    update_pose()
    l1,r1=r.enc()
    done=((l1-l0)+(r1-r0))/2.0/TICKS
    return done

def radio():
    while True:
        r.send(f'A PING {x:.1f} {y:.1f} {h:.0f}')
        t0=time.time()
        while time.time()-t0<7:
            v=r.recv(0.4)
            if v: log('RADIO RX:', v)
            time.sleep(0.2)
threading.Thread(target=radio, daemon=True).start()

log('=== explore3 start ===')
try:
    for step in range(300):
        st = r.status()
        if st and (st[1] or st[2]):
            log('FLAG!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            break
        lid = r.lidar()
        if not lid: continue
        c = clean(lid)
        # pick best direction among 16 beams: prefer unvisited target cell
        scored = []
        for i in range(16):
            az = (h + i*22.5) % 360
            rad = math.radians(az)
            probe = min(c[i], c[(i-1)%16], c[(i+1)%16])
            tx = x + 0.45*math.sin(rad); ty = y + 0.45*math.cos(rad)
            tv = ckey(tx,ty,0) in visited
            scored.append((probe, -tv, i, az))
        scored.sort(reverse=True)
        probe, negtv, best_i, best_az = scored[0]
        if probe < 0.5:
            # maybe dead end: rotate 180 and rescan
            log(f'step {step}: all blocked (best {probe:.2f}@{best_i}) pose {x:.2f},{y:.2f} h={h:.0f}')
            rotto((h+180)%360)
            continue
        if abs((best_az - h + 180)%360 - 180) > 8:
            log(f'step {step}: turn to az {best_az:.0f} (beam {best_i}, open {probe:.2f}, unvisited={negtv<0}) from {x:.2f},{y:.2f} h={h:.0f}')
            rotto(best_az)
            lid = r.lidar()
            c = clean(lid)
        d = drive(0.42)
        visited.add(ckey(x,y,h))
        if d < 0.1:
            log(f'step {step}: little progress d={d:.2f} at {x:.2f},{y:.2f} h={h:.0f}; nudge turn')
            rotto((h+60)%360)
        if step % 10 == 0:
            log(f'step {step} pose {x:.2f},{y:.2f} h={h:.0f} batt={r.battery()} visited={len(visited)}')
            json.dump({'x':x,'y':y,'h':h,'visited':len(visited),'trail':[[round(x,2),round(y,2)]]}, open('/bot/src/state3.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
