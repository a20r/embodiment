import time, math, signal, sys, json, threading, random, os
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/explore8.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0; h=r.heading() or 0.0
r.stop(); time.sleep(0.3)
le, re = r.enc()

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
            if f<0.19: break
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

def radio():
    n=0
    while True:
        _st=r.status()
        _g=_st[1] if _st else 0
        _he=_st[2] if _st else 0
        r.send(f'A PING {x:.2f} {y:.2f} {h:.0f} n={n} goal={_g} here={_he}')
        n+=1
        t0=time.time()
        while time.time()-t0<1.5:
            v=r.recv(0.12)
            if v:
                rx_count[0]+=1
                rx_last[0]=time.time()
                rx_times.append(time.time())
                hh=r.heading(); xx=x; yy=y
                log('!!!RADIO RX h=%s pose=(%.2f,%.2f):' % (hh,xx,yy), v)
                with open('/memory/rx_log.txt','a') as f:
                    f.write('%s h=%s pose=(%.2f,%.2f) %s\n' % (time.strftime('%H:%M:%S'), hh, xx, yy, v))
                try:
                    for p in ('d1','d7'):
                        fd=os.open('/dev/robot/'+p, os.O_WRONLY|os.O_NONBLOCK)
                        os.write(fd,b'0\n'); os.close(fd)
                except Exception: pass
                r.send('A ACK %d %s' % (time.time(), v.strip()[:30]))
                U=v.upper()
                if 'GOAL=1' in U or 'HERE=1' in U:
                    json.dump({'msg':v,'t':time.time()}, open('/memory/goal_from_B.json','w'))
                    b_at_goal[0]=True
                if 'B PING' in U:
                    b_seen_ct[0]=b_seen_ct[0]+1
            time.sleep(0.1)
threading.Thread(target=radio, daemon=True).start()


def detect_robot(r, clean):
    """isolated-obstacle signature: 3 adjacent beams blocked, neighbors open"""
    lid=r.lidar()
    if not lid: return None
    c=clean(lid)
    for i in range(16):
        if min(c[(i-1)%16],c[i],c[(i+1)%16])<0.5:
            if min(c[(i+3)%16],c[(i+4)%16],c[(i-3)%16],c[(i-4)%16])>0.75:
                return i
    return None

def homing(r, log, rx_count, rx_last, update_pose, drive, rotto, clean):
    """hill-climb on packets-per-3s; B beacons ~1Hz, range tiny."""
    def sample(t=3.0):
        c0=rx_count[0]; t0=time.time()
        while time.time()-t0<t:
            time.sleep(0.1)
        return rx_count[0]-c0
    best=sample()
    log('homing start count=%d' % best)
    step_i=0
    last_good=time.time()
    while step_i<500:
        step_i+=1
        st=r.status()
        if st and (st[1] or st[2]):
            log('FLAG during homing!', st)
            json.dump({'x':x,'y':y,'h':h,'flags':st,'mode':'homing'}, open('/memory/goal_found.json','w'))
            return
        # lidar pursuit: if a robot-like object is visible, chase it
        tgt=detect_robot(r,clean)
        if tgt is not None:
            hhc=r.heading() or 0
            az=(hhc+tgt*22.5)%360
            log('homing: ROBOT-SIGHTING beam=%d az=%.0f' % (tgt,az))
            rotto(az, timeout=5, tol=6)
            d=drive(0.3, timeout=6)
            c=sample(2.0)
            log('homing pursue d=%.2f count=%d' % (d,c))
            if c>best: best=c
            continue
        improved=False
        for hh in (0,45,-45,90,-90,135,-135,180):
            cur=r.heading() or 0
            rotto((cur+hh)%360, timeout=6, tol=6)
            before=rx_count[0]
            d=drive(0.25, timeout=6)
            c=sample(2.5)
            log('homing try hh=%d d=%.2f count=%d' % (hh,d,c))
            if c>best:
                best=c; improved=True
                break
            elif c==0 and best>0:
                # step back
                pass
                r.wheels(-12,-12); time.sleep(0.5); r.stop()
        if rx_count[0]>0:
            last_good=time.time()
            if len([t for t in rx_times if t>time.time()-10])<2 and best<3 and step_i>8:
                log('homing weak & sparse; resume explore')
                return
        if time.time()-last_good>60:
            log('homing lost 90s; resume explore')
            return
        if not improved:
            if best>=3:
                # strong stable contact: hold position briefly, let B approach or turn
                log('homing strong contact best=%d; holding' % best)
                time.sleep(2.0)
            else:
                log('homing plateau best=%d; keep searching' % best)

# fingerprint novelty log
try: fpsd={tuple(k):v for k,v in json.load(open('/bot/src/fpc.json'))}
except Exception: fpsd={}
fps=set(fpsd)


def zone(c):
    front=min(c[15],c[0],c[1])
    left=min(c[11],c[12],c[13])
    right=min(c[3],c[4],c[5])
    return front,left,right

rx_count=[0]; rx_last=[0.0]; rx_times=[]
b_at_goal=[False]; b_seen_ct=[0]
align={}
log('=== explore8 (left-hand) start ===')
try:
    step=0
    last_flag=t0=time.time()
    while time.time()-t0 < 100000:
        step+=1
        st=r.status()
        if st and (st[1] or st[2]):
            log('FLAG!!!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            r.stop()
            # STAY at goal and broadcast; refine position slightly if here=0 but goal=1
            import time as _t
            while True:
                st2=r.status()
                if st2 and st2[2]==0 and st2[1]==1:
                    # goal visible but not at it: nudge forward
                    d=drive(0.3)
                    if d<0.05: rotto((r.heading() or 0)+45)
                else:
                    r.stop()
                r.send(f'A AT GOAL {_t.time():.0f} flags={st2}')
                v=r.recv(0.4)
                if v:
                    log('GOAL-mode RX:', v)
                    with open('/memory/rx_log.txt','a') as f:
                        f.write('%s GOALMODE %s\n' % (time.strftime('%H:%M:%S'), v))
                time.sleep(2.5)
        _recent=[t for t in rx_times if t>time.time()-12]
        if b_at_goal[0]:
            if b_at_goal[0]:
                log('ENTER PERMANENT CHASE (B at goal!)')
            else:
                log('ENTER HOMING MODE (rx %d)' % rx_count[0])
            homing(r, log, rx_count, rx_last, update_pose, drive, rotto, clean)
            log('EXIT HOMING MODE')
            continue
        lid=r.lidar()
        if not lid: continue
        c=clean(lid)
        fp=tuple(min(9,int(v*5)) for v in c)
        new = fp not in fps
        fps.add(fp); fpsd[fp]=fpsd.get(fp,0)+1
        front,left,right=zone(c)
        acted=None
        if left>0.40:
            rotto(((h or 0)-85)%360); acted='L'
        elif front>0.30:
            acted='S'
        else:
            rotto(((h or 0)+85)%360); acted='R'
            lid2=r.lidar()
            if lid2:
                c2=clean(lid2); f2,_,_=zone(c2)
                if f2<=0.30:
                    hcur=r.heading() or h
                    rotto((hcur+85)%360); acted='RR'
        d=drive(0.5)
        if d<0.10:
            # wedged: back off
            l0,r0=r.enc(); t1=time.time()
            while time.time()-t1<2.5:
                l1,r1=r.enc()
                if l1 is None: continue
                if ((l1-l0)+(r1-r0))/2<=-0.18*TICKS: break
                r.wheels(-12,-12); time.sleep(0.04)
            r.stop()
        if step%20==0:
            # compass-aligned signature: max range per absolute azimuth bin (16 bins of 22.5deg)
            bins=[0.0]*16
            for i in range(16):
                az=int((((h+i*22.5)%360)//22.5))%16
                if c[i]>bins[az]: bins[az]=c[i]
            sig=tuple(min(9,int(v*3)) for v in bins)
            align[sig]=align.get(sig,0)+1
            rep=max(align.values())
            log(f'{step}: {acted} pose {x:.2f},{y:.2f} h={h:.0f} F={front:.2f} L={left:.2f} d={d:.2f} newfp={new} fps={len(fps)} align={len(align)} maxrep={rep}')
            json.dump([[list(k),v] for k,v in fpsd.items()], open('/bot/src/fpc.json','w'))
        if time.time()-last_flag>15:
            last_flag=time.time()
            st=r.status()
            if st and (st[1] or st[2]):
                log('FLAG-ON-TIMER!!!', st)
                json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    json.dump([[list(k),v] for k,v in fpsd.items()], open('/bot/src/fpc.json','w'))
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
