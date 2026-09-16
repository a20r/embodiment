import time, math, signal, sys, json, os, threading
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/dock.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0; h=r.heading() or 0.0
le,re = r.enc()
r.stop(); time.sleep(0.3)

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

def rotto(tg, timeout=6, tol=6):
    t0=time.time()
    while time.time()-t0<timeout:
        hc=r.heading()
        if hc is None: continue
        e=(tg-hc+180)%360-180
        if abs(e)<=tol: break
        v=max(12,min(34,abs(e)*1.3))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.08)

def drive_fwd(dist, speed=18, timeout=6):
    """open-loop-ish forward with front safety"""
    l0,r0=r.enc()
    if l0 is None: return 0.0
    t0=time.time()
    while time.time()-t0<timeout:
        l1,r1=r.enc()
        if l1 is None: continue
        prog=((l1-l0)+(r1-r0))/2.0
        if prog>=dist*TICKS: break
        lid=r.lidar()
        if lid:
            c=clean(lid)
            f=min(c[15],c[0],c[1])
            if f<0.22: break
        r.wheels(speed,speed); time.sleep(0.04)
    r.stop(); time.sleep(0.05)
    l1,r1=r.enc()
    return ((l1-l0)+(r1-r0))/2.0/TICKS

def scan_clusters():
    """returns list of (center_beam, min_range) for contiguous blocked clusters"""
    lid=r.lidar()
    if not lid: return []
    c=clean(lid)
    blocked=[i for i in range(16) if c[i]<0.55]
    clusters=[]
    used=set()
    for i in blocked:
        if i in used: continue
        grp=[i]
        used.add(i)
        # extend both directions (circular)
        j=(i+1)%16
        while j in blocked and j not in used:
            grp.append(j); used.add(j); j=(j+1)%16
        j=(i-1)%16
        while j in blocked and j not in used:
            grp.append(j); used.add(j); j=(j-1)%16
        # center beam = closest
        cen=min(grp, key=lambda k: c[k])
        clusters.append((cen, c[cen], len(grp)))
    return clusters

rx_state={'count':0,'last':0.0,'bgoal':False,'bhere':False,'bpose':None}
def radio():
    n=0
    while True:
        _st=r.status()
        r.send('A PING %.2f %.2f %.0f n=%d goal=%d here=%d' % (x,y,h,n,_st[1] if _st else 0,_st[2] if _st else 0))
        n+=1
        t0=time.time()
        while time.time()-t0<1.0:
            v=r.recv(0.06)
            if v:
                rx_state['count']+=1
                rx_state['last']=time.time()
                s=v.strip()
                U=s.upper()
                if U.startswith('B PING'):
                    try:
                        p=s.split()
                        rx_state['bpose']=(float(p[2]),float(p[3]),float(p[4]))
                        rx_state['bgoal']='GOAL=1' in U
                        rx_state['bhere']='HERE=1' in U
                        if rx_state['bgoal'] or rx_state['bhere']:
                            json.dump({'msg':s,'t':time.time()}, open('/memory/goal_from_B.json','w'))
                    except Exception: pass
                with open('/memory/rx_log.txt','a') as f:
                    f.write('%s %.2f %.2f %s\n' % (time.strftime('%H:%M:%S'), x, y, s[:60]))
threading.Thread(target=radio, daemon=True).start()

log('=== dock start === pose(%.2f,%.2f) h=%.0f' % (x,y,h))
prev_clusters=[]
lost_since=time.time()
try:
    while True:
        # MY flags first
        st=r.status()
        if st and (st[1] or st[2]):
            log('MY FLAG!!!', st)
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            r.stop()
            while True:
                _st=r.status()
                r.send('A AT GOAL %d flags=%s' % (time.time(), _st))
                v=r.recv(0.3)
                if v:
                    log('goal-mode RX:', v)
                    with open('/memory/rx_log.txt','a') as f:
                        f.write('%s GOALMODE %s\n' % (time.strftime('%H:%M:%S'), v))
                time.sleep(2.0)
        # B at goal?
        if rx_state['bgoal'] or rx_state['bhere']:
            log('B AT GOAL -> shadow hard! bpose=%s' % (rx_state['bpose'],))
        # radio contact status
        linked = (time.time()-rx_state['last']) < 4.0
        update_pose()
        cl=scan_clusters()
        # find moving cluster: compare with previous by center proximity & range change
        target=None
        for cen,rng,sz in cl:
            for pc,pr,psz in prev_clusters:
                if min((cen-pc)%16,(pc-cen)%16) <= 2 and abs(rng-pr)>0.08 and rng>0.12:
                    target=(cen,rng); break
            if target: break
        prev_clusters=cl
        if target:
            cen,rng=target
            hh=r.heading() or 0
            az=(hh+cen*22.5)%360
            log('target beam=%d az=%.0f rng=%.2f linked=%s' % (cen,az,rng,linked))
            rotto(az, timeout=4, tol=6)
            if rng>0.5:
                drive_fwd(min(0.25, rng-0.30), speed=16, timeout=4)
            # if close, wait
            elif rng<0.35:
                time.sleep(0.6)
            else:
                time.sleep(0.2)
        elif linked:
            # no moving object visible but linked: gentle drift toward last target/hold
            time.sleep(0.3)
        else:
            # lost: rotate-scan search
            rotto(((r.heading() or 0)+45)%360, timeout=4, tol=8)
            if time.time()-lost_since>240:
                log('dock: 4min no contact -> resume exploring (exit)')
                break
            lost_since=lost_since if (time.time()-rx_state['last']<240) else time.time()
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    log('=== dock end ===')
