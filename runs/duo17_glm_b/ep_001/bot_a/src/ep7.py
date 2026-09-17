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

def d11avg(n=8):
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
    try: e['d3']=rd('d3',10)[:48]
    except Exception: pass
    e.update(kw)
    log.append(e)
    print('%s %s'%(tag,json.dumps({k:e.get(k) for k in ('d4','d11','d0','d5','eL','eR')})),flush=True)
    json.dump(log, open('/memory/ep7.json','w'))
    return e

MSGS = ['READY','A ALIVE HDG=198','POS UNKNOWN d11=HIGH','SEE','GOAL NO STUCK NO','A HOLDING HERE COME','HDG=198 d11 GOOD']
def beacon():
    i = 0
    while not stop:
        m = MSGS[i%len(MSGS)]
        tx(m)
        i += 1
        time.sleep(0.7)

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
t0 = time.time(); ready_sent = 0
while time.time()-t0 < 12000:
    e = S('hold', ready=ready_sent)
    if e.get('d0')=='1':
        print('GOAL!!',flush=True)
        for _ in range(30):
            tx('GOAL YES A AT GOAL COME FAST'); time.sleep(2)
    cur = e['d11']
    if cur > 0.9:
        # heavy READY burst while B adjacent
        print('NEAR d11=%.3f spam READY'%cur,flush=True)
        for _ in range(10):
            tx('READY A HERE d11>0.9 STOP NOW'); time.sleep(1.5)
    elif ready_sent and cur < 0.6:
        ready_sent = 0
    if cur > 0.9 and not ready_sent:
        ready_sent = 1
    time.sleep(2)
d.set(0,0)
d.close()
json.dump(log, open('/memory/ep7.json','w'))
print('DONE',flush=True)
