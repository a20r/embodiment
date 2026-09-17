import json, time, threading, os, sys, math
sys.path.insert(0,'/bot/src')
from driver import Driver, rd, lidar

d = Driver()
log = []
stop = False
best = 0.0

def tx(msg):
    try:
        fd = os.open('/dev/robot/d8', os.O_WRONLY)
        os.write(fd, msg.encode()+b'\n'); os.close(fd)
    except Exception:
        pass

def d11avg(n=5):
    vs=[]
    for _ in range(n):
        try: vs.append(float(rd('d11',10)))
        except Exception: pass
        time.sleep(0.07)
    return sum(vs)/max(1,len(vs))

def S(tag, **kw):
    e = {'tag':tag,'t':round(time.time(),1)}
    for p,k in (('d4','d4'),('d0','d0'),('d5','d5'),('d9','eL'),('d6','eR')):
        try: e[k]=rd(p,10)[:24]
        except Exception: pass
    e['d11']=round(d11avg(5),3)
    e.update(kw)
    log.append(e)
    print('%s %s'%(tag,json.dumps({k:e.get(k) for k in ('d4','d11','d0','d5','eL','eR')})),flush=True)
    json.dump(log, open('/memory/ep18.json','w'))
    return e

MSGS = ['GOAL YES A COMING','READY','A COMING TO B GREEDY','SEE','A COME','HDG=198','GOAL YES A ON WAY','A TX CONTINUOUS']
def beacon():
    i = 0
    while not stop:
        tx(MSGS[i%len(MSGS)]); i += 1; time.sleep(0.7)

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

def backup(sec=2.5):
    d.set(-1,-1); time.sleep(sec); d.set(0,0); time.sleep(0.3)

def sidemin(l, idxs):
    xs=[l[i] for i in idxs if l[i]>0]
    return min(xs) if xs else 9.9

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start-greedy2')
t0=time.time(); n=0; lastdir=1; consecbad=0; consecblocked=0
GREEDY_ON=0.70
while time.time()-t0 < 12000:
    n+=1
    e0 = S('leg%d-a'%n)
    if e0.get('d0')=='1' or e0.get('d5')=='1' or e0['d11']>0.92:
        print('ARRIVAL? d0=%s d5=%s d11=%.3f'%(e0.get('d0'),e0.get('d5'),e0['d11']),flush=True)
        d.set(0,0)
        for k in range(40):
            tx('A AT GOAL GOAL YES HERE'); time.sleep(2)
            e=S('arr%d'%k)
            if e['d11']<0.6 and e.get('d0')=='0' and e.get('d5')=='0': break
        continue
    cur=e0['d11']; best=max(best,cur)
    l = lidar()
    front = min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    right = sidemin(l,(4,5,6)); left = sidemin(l,(10,11,12))
    if cur>=GREEDY_ON:
        # near B: fine greedy
        if front>0.32: drive(12)
        else:
            consecblocked+=1
            if consecblocked>=3: backup(); consecblocked=0
            if right>=left: turn(+75)
            else: turn(-75)
            drive(6)
        e1=S('leg%d-b'%n)
        dd=e1['d11']-e0['d11']
        print('FINE dd=%+.3f best=%.3f'%(dd,best),flush=True)
        if dd<-0.012:
            lastdir=-lastdir; turn(50*lastdir)
        continue
    # far: coarse greedy 30s legs
    if front>0.34:
        consecblocked=0
        drive(30)
    else:
        consecblocked+=1
        if consecblocked>=3: backup(); consecblocked=0
        if right>=left: turn(+80)
        else: turn(-80)
        drive(8)
    e1=S('leg%d-b'%n)
    dd=e1['d11']-e0['d11']
    print('GREEDY2 dd=%+.3f bad=%d best=%.3f'%(dd,consecbad,best),flush=True)
    if dd<-0.010:
        consecbad+=1
        if consecbad>=4:
            turn(180*lastdir); consecbad=0; S('turn180')
        else:
            lastdir=-lastdir
            turn(75*lastdir); S('turn75',dir=lastdir)
    elif dd>0.004:
        consecbad=0
    else:
        turn(30*lastdir); S('turn30')

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep18.json','w'))
print('DONE',flush=True)
