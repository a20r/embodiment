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
    json.dump(log, open('/memory/ep10.json','w'))
    return e

def beacon():
    i = 0
    while not stop:
        tx(MSGS[i%len(MSGS)]); i += 1; time.sleep(0.7)
MSGS = ['READY','A ALIVE A HOLDING','POS UNKNOWN','SEE','GOAL NO STUCK NO','A HERE COME','HDG=198','BEAM? ASK AGAIN']

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

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start-hold')
t0=time.time(); low_since=None; lastlog=0
while time.time()-t0 < 12000:
    e = S('hold')
    if e.get('d0')=='1':
        print('GOAL!!',flush=True)
        for _ in range(30): tx('GOAL YES A AT GOAL COME FAST'); time.sleep(2)
    cur = e['d11']
    if e.get('d5')=='1' or cur>0.92:
        print('NEAR: READY spam d11=%.3f d5=%s'%(cur,e.get('d5')),flush=True)
        for k in range(10):
            tx('READY SEE A HERE STOP NOW d11=%.2f'%cur); time.sleep(2)
            e=S('near%d'%k)
            if e['d11']<0.7 and e.get('d5')=='0': break
    # fallback: if B lost for >12min, resume approach
    if cur < 0.55:
        if low_since is None: low_since = time.time()
        elif time.time()-low_since > 720:
            print('B LOST 12min: brief approach burst',flush=True)
            d.set(1,1); time.sleep(25); d.set(0,0)
            e2=S('approach-burst')
            if e2['d11'] < cur-0.01:
                d.set(1,-1); time.sleep(4.0); d.set(0,0); time.sleep(0.3)
            low_since = time.time()
    else:
        low_since = None

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep10.json','w'))
print('DONE',flush=True)
