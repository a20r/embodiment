import json, time, threading, os, sys
sys.path.insert(0,'/bot/src')
from driver import Driver, rd, lidar

d = Driver()
log = []
stop = False

def tx(msg):
    try:
        fd = os.open('/dev/robot/d8', os.O_WRONLY)
        os.write(fd, msg.encode()+b'\n'); os.close(fd)
    except Exception:
        pass

def d11avg(n=6):
    vs=[]
    for _ in range(n):
        try: vs.append(float(rd('d11',10)))
        except Exception: pass
        time.sleep(0.08)
    return sum(vs)/max(1,len(vs))

def S(tag, **kw):
    e = {'tag':tag,'t':round(time.time(),1)}
    for p,k in (('d4','d4'),('d0','d0'),('d5','d5'),('d9','eL'),('d6','eR')):
        try: e[k]=rd(p,10)[:24]
        except Exception: pass
    e['d11']=round(d11avg(6),3)
    e.update(kw)
    log.append(e)
    print('%s %s'%(tag,json.dumps({k:e.get(k) for k in ('d4','d11','d0','d5','eL','eR')})),flush=True)
    json.dump(log, open('/memory/ep11.json','w'))
    return e

def beacon():
    i = 0
    while not stop:
        tx(MSGS[i%len(MSGS)] if MSGS else 'A BEACON'); i += 1; time.sleep(0.7)
MSGS = ['READY','A ALIVE A MOVING TO YOU','POS UNKNOWN d11 RISING','SEE','GOAL NO STUCK NO','A COME TO ME','HDG=198']

def rxloop():
    while not stop:
        try:
            with open('/dev/robot/d10') as f:
                v = f.read().strip()
            if v:
                print('RX %s'%v, flush=True)
                log.append({'tag':'RX','t':round(time.time(),1),'msg':v})
        except Exception:
            time.sleep(0.5)

def turn(deg_cw):
    t = abs(deg_cw)/16.0
    d.set(1,-1) if deg_cw>0 else d.set(-1,1)
    time.sleep(t); d.set(0,0); time.sleep(0.3)

def drive(sec):
    d.set(1,1); time.sleep(sec); d.set(0,0); time.sleep(0.4)

def bounce():
    l = lidar()
    L = sum(x for x in l[10:16] if x>0); R = sum(x for x in l[0:6] if x>0)
    if R<=L: turn(+90)
    else: turn(-90)

def blob():
    # isolated robot-sized reading: ray k short, neighbors much longer
    l = lidar()
    best = None
    for k in range(16):
        v = l[k]
        if 0.2 < v < 0.85:
            a = l[(k-1)%16]; b = l[(k+1)%16]
            aa = a if a>0 else 5.0; bb = b if b>0 else 5.0
            if min(aa,bb) > v+0.45:
                score = min(aa,bb)-v
                if best is None or score>best[0]: best=(score,k,v)
    return best  # (score, ray, dist) or None

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start')
t0=time.time(); ema=None; consecbad=0; best=0.0; n=0; lastdir=1
while time.time()-t0 < 12000:
    n+=1
    e0 = S('leg%d-a'%n)
    if e0.get('d0')=='1': print('GOAL!!',flush=True); break
    if e0.get('d5')=='1':
        print('D5 B-NEAR: SEE+ready',flush=True)
        d.set(0,0)
        for k in range(8):
            tx('SEE READY A HERE d11=%.2f'%e0['d11']); time.sleep(3)
            try:
                if rd('d5')=='0': break
            except Exception: pass
        continue
    cur = e0['d11']; best=max(best,cur)
    if cur>0.92:
        print('CONTACT d11>0.92 hold+ready',flush=True)
        d.set(0,0)
        for k in range(12):
            tx('READY A HERE STOP NOW'); time.sleep(2)
            e=S('contact%d'%k)
            if e['d11']<0.75: break
        ema=None
        continue
    bl = blob()
    if bl:
        sc,k,v = bl
        try: hdg = rd('d4',10)
        except Exception: hdg='?'
        tx('BEAM%d HDG=%s SEE'%(k,hdg))
        print('BLOB ray%d d=%.2f score=%.2f'%(k,v,sc),flush=True)
        # visual servo: rotate so blob moves toward ray0, then creep forward
        if k <= 8:
            turn(+int(k*22.5*0.8)) if k>0 else None
        else:
            turn(-int((16-k)*22.5*0.8))
        time.sleep(0.3)
        l2 = lidar()
        if min([x for x in (l2[15],l2[0],l2[1]) if x>0]+[99])>0.30:
            drive(6); S('servo')
        else:
            drive(2)
        continue
    l = lidar()
    head = min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    if head>0.32:
        drive(15)
    else:
        bounce()
        drive(12)
    e1=S('leg%d-b'%n)
    if e1.get('d0')=='1': print('GOAL!!',flush=True); break
    if e1.get('d5')=='1': continue
    dd=e1['d11']-e0['d11']
    ema = dd if ema is None else 0.6*ema+0.4*dd
    print('LEG%d dd=%+.3f ema=%+.3f bad=%d best=%.3f'%(n,dd,ema,consecbad,best),flush=True)
    if ema<-0.010:
        consecbad+=1
        if consecbad>=3:
            turn(180*lastdir); consecbad=0; S('turn180')
        else:
            lastdir = -lastdir
            turn(90*lastdir); S('turn90',dir=lastdir)
        ema=None
    elif ema>0.008:
        consecbad=0
    else:
        turn(45*lastdir); S('turn45'); ema=None

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep11.json','w'))
print('DONE',flush=True)
