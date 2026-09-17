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
    json.dump(log, open('/memory/ep15.json','w'))
    return e

def beacon():
    i = 0
    while not stop:
        tx(MSGS[i%len(MSGS)]); i += 1; time.sleep(0.7)
MSGS = ['GOAL YES A COMING','READY','A DRIVE TO B d11 RISING','SEE','A COME ON WAY','HDG=198','GOAL YES A ON WAY','A TX CONTINUOUS']

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

S('start-race')
t0=time.time(); n=0; lastdir=1; consecbad=0
while time.time()-t0 < 12000:
    n+=1
    e0 = S('leg%d-a'%n)
    # arrival checks
    if e0.get('d0')=='1' or e0.get('d5')=='1' or e0['d11']>0.92:
        print('ARRIVAL? d0=%s d5=%s d11=%.3f'%(e0.get('d0'),e0.get('d5'),e0['d11']),flush=True)
        d.set(0,0)
        for k in range(40):
            tx('A AT GOAL GOAL YES HERE'); time.sleep(2)
            e=S('arr%d'%k)
            if e['d11']<0.6 and e.get('d0')=='0' and e.get('d5')=='0':
                break
        continue
    l = lidar()
    head = min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    if head>0.32: drive(18)
    else:
        bounce(); drive(10)
    e1=S('leg%d-b'%n)
    dd=e1['d11']-e0['d11']
    print('RACE dd=%+.3f bad=%d'%(dd,consecbad),flush=True)
    if dd<-0.015:
        consecbad+=1
        if consecbad>=4:
            turn(180*lastdir); consecbad=0; S('turn180')
        else:
            lastdir=-lastdir; turn(70*lastdir); S('turn70',dir=lastdir)
    elif dd>0.005:
        consecbad=0
    else:
        turn(35*lastdir); S('turn35')

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep15.json','w'))
print('DONE',flush=True)
