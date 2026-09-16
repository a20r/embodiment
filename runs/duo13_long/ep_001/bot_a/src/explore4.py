import time, math, signal, sys, json, threading, random
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/explore4.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0
h = r.heading() or 0.0
r.stop(); time.sleep(0.3)
le, re = r.enc()

CELL=0.35
visited=set()
blacklist={}   # (cellkey, az_octant) -> count

def update_pose():
    global x,y,h,le,re
    l2,r2=r.enc()
    if l2 is None: return
    h2=r.heading()
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
    global x,y,h
    l0,r0=r.enc()
    if l0 is None: return 0.0
    t0=time.time()
    while time.time()-t0<timeout:
        update_pose()
        l1,r1=r.enc()
        if l1 is None: continue
        prog=((l1-l0)+(r1-r0))/2.0
        if prog>=dist_max*TICKS: break
        lid=r.lidar()
        if lid:
            c=clean(lid)
            f=min(c[15],c[0],c[1])
            if f<0.20: break
            sp=22 if f>0.4 else 12
            err=(min(c[12],c[13])-min(c[3],c[4]))
            corr=max(-7,min(7,err*20))
            r.wheels(sp-corr,sp+corr)
        else:
            r.wheels(15,15)
        time.sleep(0.03)
    r.stop(); time.sleep(0.1)
    update_pose()
    l1,r1=r.enc()
    return ((l1-l0)+(r1-r0))/2.0/TICKS

def radio():
    n=0
    while True:
        r.send(f'A PING {x:.1f} {y:.1f} {h:.0f} n={n}')
        n+=1
        t0=time.time()
        while time.time()-t0<7:
            v=r.recv(0.4)
            if v: log('RADIO RX:', v)
            time.sleep(0.2)
threading.Thread(target=radio, daemon=True).start()

def cell_of(px,py): return (int(round(px/CELL)), int(round(py/CELL)))

log('=== explore4 start ===')
try:
    last_progress=time.time()
    for step in range(500):
        st=r.status()
        if st and (st[1] or st[2]):
            log('FLAG!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            break
        ck=cell_of(x,y)
        visited.add(ck)
        lid=r.lidar()
        if not lid: continue
        c=clean(lid)
        # score directions by octant (8 dirs)
        opts=[]
        for o in range(8):
            az=(h+o*45)%360
            # beams near this octant: center beam index = o*2
            idx=int(round(o*2))%16
            probe=min(c[idx], c[(idx-1)%16], c[(idx+1)%16])
            tv = cell_of(x+0.5*math.sin(math.radians(az)), y+0.5*math.cos(math.radians(az))) in visited
            bl = blacklist.get((ck,o),0)
            opts.append((probe, -tv-2*bl, o, az))
        opts.sort(reverse=True)
        probe, sc, o, az = opts[0]
        if probe < 0.5 or sc <= -6:
            log(f'step {step}: escape at {x:.2f},{y:.2f} h={h:.0f} (probe={probe:.2f})')
            rotto((h+180)%360)
            d=drive(0.22)
            if d<0.08:
                rotto((h+90)%360); drive(0.15)
            continue
        if abs((az-h+180)%360-180)>8:
            log(f'step {step}: -> az {az:.0f} oct{o} probe={probe:.2f} sc={sc} at {x:.2f},{y:.2f}')
            rotto(az)
        d=drive(0.42)
        if d<0.08:
            blacklist[(ck,o)]=blacklist.get((ck,o),0)+1
            log(f'step {step}: blocked az={az:.0f} count={blacklist[(ck,o)]} at {x:.2f},{y:.2f}')
        else:
            last_progress=time.time()
            blacklist.pop((ck,o),None)
        if time.time()-last_progress>60:
            log('STUCK 60s: random shuffle')
            rotto((h+random.choice([90,-90,135]))%360); drive(0.15)
            last_progress=time.time()
        if step%10==0:
            log(f'step {step} pose {x:.2f},{y:.2f} h={h:.0f} batt={r.battery()} visited={len(visited)}')
            json.dump({'x':x,'y':y,'h':h,'visited':sorted(visited)}, open('/bot/src/state4.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
