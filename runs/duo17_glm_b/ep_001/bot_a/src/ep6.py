import json, time, math, threading, os, sys
sys.path.insert(0,'/bot/src')
from driver import Driver, rd, lidar

d = Driver()
log = []
stop = False
STATE = {'d11': 0.0, 'phase': 'approach'}

def tx(msg):
    try:
        fd = os.open('/dev/robot/d8', os.O_WRONLY)
        os.write(fd, msg.encode()+b'\n'); os.close(fd)
    except Exception:
        pass

TOKENS = ['READY','SEE','A ALIVE','GOAL NO','STUCK NO','READY A MOVING TO YOU','NOTSEE','GOAL YES']
def beacon():
    i = 0
    while not stop:
        m = TOKENS[i%len(TOKENS)]
        tx(m)
        try: st = rd('d3',5)
        except Exception: st='?'
        print('TX %r -> %s' % (m, st), flush=True)
        log.append({'tag':'TX','t':round(time.time(),1),'msg':m,'st':st})
        json.dump(log, open('/memory/ep6.json','w'))
        i += 1
        time.sleep(5)

def rxloop():
    while not stop:
        try:
            with open('/dev/robot/d10') as f:
                v = f.read().strip()
            if v:
                print('RX %s'%v, flush=True)
                log.append({'tag':'RX','t':round(time.time(),1),'msg':v})
        except Exception:
            time.sleep(1)

def d11avg(n=10):
    vs=[]
    for _ in range(n):
        try: vs.append(float(rd('d11',10)))
        except Exception: pass
        time.sleep(0.12)
    return sum(vs)/max(1,len(vs))

def S(tag, **kw):
    e = {'tag':tag,'t':round(time.time(),1)}
    try: e['d4']=round(float(rd('d4',10)),1)
    except Exception: e['d4']=-1
    e['d11']=round(d11avg(8),3)
    try:
        e['eL']=int(float(rd('d9',10))); e['eR']=int(float(rd('d6',10)))
    except Exception: pass
    try: e['d0']=rd('d0',10)
    except Exception: e['d0']='?'
    try: e['d5']=rd('d5',10)
    except Exception: e['d5']='?'
    try: e['d3']=rd('d3',10)[:60]
    except Exception: pass
    e.update(kw)
    log.append(e); STATE['d11']=e['d11']
    print('%s %s'%(tag,json.dumps({k:e.get(k) for k in ('d4','d11','eL','eR','d0','d5')})),flush=True)
    json.dump(log, open('/memory/ep6.json','w'))
    return e

def turn(deg_cw):
    # ~16 deg/s spin
    t = abs(deg_cw)/16.0
    if deg_cw>0: d.set(1,-1)
    else: d.set(-1,1)
    time.sleep(t); d.set(0,0); time.sleep(0.3)

def drive(sec):
    d.set(1,1); time.sleep(sec); d.set(0,0); time.sleep(0.4)

def bounce():
    # turn away from more-blocked side, then a short drive
    l = lidar()
    L = sum(x for x in l[10:16] if x>0); R = sum(x for x in l[0:6] if x>0)
    if R<=L: turn(+90)
    else: turn(-90)

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start')
t0=time.time(); ema=None; lastturn=0; consecbad=0; best=0.0
n=0
while time.time()-t0 < 12000:
    n+=1
    e0 = S('leg%d-a'%n)
    if e0['d0']=='1': STATE['phase']='GOAL'; break
    # B adjacent?
    if e0['d5']=='1':
        print('D5=1 B NEAR',flush=True)
        d.set(0,0)
        for k in range(6):
            tx('A-TO-B: READY SEE d11=%.2f A HOLDING'%e0['d11']); time.sleep(4)
            e=S('d5hold%d'%k)
            if e['d5']=='0': break
    cur=e0['d11']; best=max(best,cur)
    if cur>0.95:
        STATE['phase']='contact'; print('CONTACT d11>0.95',flush=True); break
    head = min([x for x in (lidar()[15],)+tuple(lidar()[0:2]) if x>0]+[99])
    if head>0.34:
        drive(60 if cur<0.6 else 35)
    else:
        bounce()
        drive(35)
    e1=S('leg%d-b'%n)
    if e1['d0']=='1': STATE['phase']='GOAL'; break
    if e1['d5']=='1': continue
    if e1['d11']>0.95:
        STATE['phase']='contact'; print('CONTACT d11>0.95',flush=True); break
    dd=e1['d11']-e0['d11']
    ema = dd if ema is None else 0.6*ema+0.4*dd
    print('LEG%d dd=%+.3f ema=%+.3f lastturn=%d bad=%d best=%.3f'%(n,dd,ema,lastturn,consecbad,best),flush=True)
    if ema<-0.012:
        consecbad+=1
        if consecbad>=3:
            turn(180); lastturn=0; consecbad=0; S('turn180')
        else:
            nd = 90 if lastturn>=0 else -90
            turn(nd); lastturn=nd; S('turn90',dir=nd)
        ema=None
    elif ema>0.008:
        consecbad=0  # keep heading
    else:
        turn(45 if lastturn>=0 else -45); S('turn45'); ema=None

# final phase: hold & spam
print('PHASE',STATE['phase'],flush=True)
t1=time.time()
while time.time()-t1<6000:
    e=S('hold')
    if STATE['phase']=='GOAL':
        tx('A-TO-B: GOAL YES A AT GOAL COME TO ME'); time.sleep(3)
    else:
        tx('A-TO-B: READY A HOLDING d11=%.2f'%e['d11']); time.sleep(6)
    if e['d0']=='1' and STATE['phase']!='GOAL':
        STATE['phase']='GOAL'
d.set(0,0)
d.close()
json.dump(log, open('/memory/ep6.json','w'))
print('DONE',flush=True)
