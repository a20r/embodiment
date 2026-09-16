import time, math, signal, sys, json, threading, random
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/explore7.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0; h = r.heading() or 0.0
r.stop(); time.sleep(0.3)
le, re = r.enc()
recent=[]  # recent cell keys
CELL=0.4

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
            cc=[z for z in (a,b) if z and z>0]
            v=min(cc) if cc else 0.4
        out.append(v)
    return out

def rotto(tg, timeout=8, tol=6):
    t0=time.time()
    while time.time()-t0<timeout:
        hc=r.heading()
        if hc is None: continue
        e=(tg-hc+180)%360-180
        if abs(e)<=tol: break
        v=max(12,min(38,abs(e)*1.4))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.1)

def drive(dist_max, timeout=10):
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
            corr=max(-6,min(6,err*18))
            r.wheels(sp-corr,sp+corr)
        else:
            r.wheels(15,15)
        time.sleep(0.03)
    r.stop(); time.sleep(0.08)
    update_pose()
    l1,r1=r.enc()
    return ((l1-l0)+(r1-r0))/2.0/TICKS

def cellk(): return (int(round(x/CELL)), int(round(y/CELL)))

def radio():
    n=0
    while True:
        r.send(f'A PING {x:.1f} {y:.1f} {h:.0f} n={n}')
        n+=1
        t0=time.time()
        while time.time()-t0<6:
            v=r.recv(0.15)
            if v:
                log('!!!RADIO RX:', v)
                with open('/memory/rx_log.txt','a') as f:
                    f.write('%s %s\n' % (time.strftime('%H:%M:%S'), v))
                U=v.upper()
                if 'GOAL' in U:
                    json.dump({'msg':v,'t':time.time()}, open('/memory/goal_from_B.json','w'))
            time.sleep(0.1)
threading.Thread(target=radio, daemon=True).start()

log('=== explore7 start ===')
try:
    for step in range(1000):
        st=r.status()
        if st and (st[1] or st[2]):
            log('FLAG!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            break
        update_pose()
        ck=cellk()
        if not recent or recent[-1]!=ck: recent.append(ck)
        if len(recent)>14: recent.pop(0)
        lid=r.lidar()
        if not lid: continue
        c=clean(lid)
        opts=[]
        for o in range(8):
            az=(h+o*45)%360
            idx=int(round(o*2))%16
            probe=min(c[idx], c[(idx-1)%16], c[(idx+1)%16])
            rad=math.radians(az)
            fk=(int(round((x+0.5*math.sin(rad))/CELL)), int(round((y+0.5*math.cos(rad))/CELL)))
            rec = fk in recent
            back = (o==4)
            opts.append((probe, -rec*1.5 - back*3.0, o, az, probe+random.uniform(0,0.05)))
        opts.sort(key=lambda t:(-t[4], t[1]))
        probe, pen, o, az, jitter = opts[0]
        if probe<0.33 and not (len(opts)>1 and opts[1][0]>0.33):
            log(f'{step}: dead end at {x:.2f},{y:.2f} h={h:.0f}; U-turn')
            rotto((h+180)%360)
            drive(0.18)
            continue
        if abs((az-h+180)%360-180)>8:
            rotto(az)
        d=drive(0.62)
        if step%25==0:
            log(f'{step}: pose {x:.2f},{y:.2f} h={h:.0f} d={d:.2f} probe={probe:.2f} batt={r.battery()} recent={len(recent)}')
            json.dump({'x':x,'y':y,'h':h,'recent':recent}, open('/bot/src/state7.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
