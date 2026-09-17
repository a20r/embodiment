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
    json.dump(log, open('/memory/ep14.json','w'))
    return e

MSGS = ['READY','A ALIVE A PATROLLING','POS UNKNOWN','SEE','GOAL NO STUCK NO','A HERE COME','HDG=198','A TX CONTINUOUS']
def bestblob():
    try:
        l = lidar(); bi=None; bs=-9
        for k in range(16):
            v=l[k]
            if 0.2<v<0.9:
                a=l[(k-1)%16]; b=l[(k+1)%16]
                aa=a if a>0 else 5.0; bb=b if b>0 else 5.0
                s=min(aa,bb)-v
                if s>bs: bs=s; bi=k
        return bi
    except Exception:
        return None
def beacon():
    i = 0
    while not stop:
        if i%8==5:
            k=bestblob()
            if k is not None:
                try: h=rd('d4',10)
                except Exception: h='?'
                tx('BEAM%d HDG=%s SEE'%(k,h))
            else:
                tx('SEE NOBEAM A MOVING')
        else:
            tx(MSGS[i%len(MSGS)])
        i += 1; time.sleep(0.7)

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

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start')
t0=time.time(); n=0; lastdir=1; best=0.0
FREEZE=0.72
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
        print('CONTACT hold+ready',flush=True)
        d.set(0,0)
        for k in range(12):
            tx('READY A HERE STOP NOW'); time.sleep(2)
            e=S('contact%d'%k)
            if e['d11']<0.75: break
        continue
    if cur>=FREEZE:
        print('FREEZE d11=%.3f B homing'%cur,flush=True)
        high=0
        for k in range(8):
            time.sleep(5)
            e=S('fz%d'%k)
            if e.get('d5')=='1' or e['d11']>0.92:
                for j in range(10):
                    tx('READY SEE A HERE STOP NOW'); time.sleep(2)
                    e2=S('near%d'%j)
                    if e2['d11']<0.7 and e2.get('d5')=='0': break
                break
            if e['d11']<0.60: break
            if e['d11']>0.78: high+=1
            else: high=0
            if high>=3:
                l = lidar()
                head = min([x for x in (l[15],l[0],l[1]) if x>0]+[99])
                if head>0.35:
                    print('CREEP forward',flush=True)
                    d.set(1,1); time.sleep(3); d.set(0,0); time.sleep(0.3)
        continue
    # patrol drive: prefer direction of recent +dd; bounce on block
    l = lidar()
    head = min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    if head>0.32: drive(16)
    else:
        bounce(); drive(10)
    e1=S('leg%d-b'%n)
    if e1.get('d0')=='1': print('GOAL!!',flush=True); break
    if e1.get('d5')=='1': continue
    dd=e1['d11']-e0['d11']
    print('PATROL dd=%+.3f best=%.3f'%(dd,best),flush=True)
    if dd<-0.03:
        lastdir=-lastdir
        turn(60*lastdir); S('turn60',dir=lastdir)
    elif dd>0.03:
        pass  # keep heading
    # flat: keep heading

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep14.json','w'))
print('DONE',flush=True)
