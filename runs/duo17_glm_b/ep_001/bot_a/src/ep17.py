import json, time, threading, os, sys, math
sys.path.insert(0,'/bot/src')
from driver import Driver, rd, lidar

d = Driver()
log = []
stop = False
hist = []   # (d4deg, dd11, dist)

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
    json.dump(log, open('/memory/ep17.json','w'))
    return e

MSGS = ['GOAL YES A COMING','READY','A SEARCHING TO B','SEE','A COME ON WAY','HDG=198','GOAL YES A ON WAY','A TX CONTINUOUS']
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

def fitbearing():
    if len(hist)<8: return None, 0.0
    M=[[0.0]*3 for _ in range(3)]; V=[0.0]*3
    for (h,dd,dist) in hist[-60:]:
        x=math.cos(math.radians(h)); y=math.sin(math.radians(h)); w=max(dist,50)
        row=[x,y,1.0]
        for i in range(3):
            for j in range(3): M[i][j]+=w*row[i]*row[j]
            V[i]+=w*row[i]*dd
    for i in range(3):
        p=max(range(i,3),key=lambda r:abs(M[r][i])); M[i],M[p]=M[p],M[i]; V[i],V[p]=V[p],V[i]
        for r in range(i+1,3):
            f=M[r][i]/M[i][i]
            for cc in range(i,3): M[r][cc]-=f*M[i][cc]
            V[r]-=f*V[i]
    sol=[0]*3
    for i in (2,1,0):
        s=V[i]-sum(M[i][j]*sol[j] for j in range(i+1,3))
        try: sol[i]=s/M[i][i]
        except Exception: return None,0.0
    a,b,c=sol
    mag=math.hypot(a,b)
    if mag<1e-6: return None,0.0
    return math.degrees(math.atan2(b,a))%360, mag

def avg(l, idxs):
    xs=[l[i] for i in idxs if l[i]>0]
    return sum(xs)/len(xs) if xs else 9.9

threading.Thread(target=beacon,daemon=True).start()
threading.Thread(target=rxloop,daemon=True).start()

S('start-pursuit')
t0=time.time(); n=0; consec_blocked=0
GREEDY=0.75
while time.time()-t0 < 12000:
    n+=1
    e0 = S('leg%d-a'%n)
    if n%7==0:
        tx('A AT d11=%.2f BRG BOUND'%e0['d11'])
    if e0.get('d0')=='1' or e0.get('d5')=='1' or e0['d11']>0.92:
        print('ARRIVAL? d0=%s d5=%s d11=%.3f'%(e0.get('d0'),e0.get('d5'),e0['d11']),flush=True)
        d.set(0,0)
        for k in range(40):
            tx('A AT GOAL GOAL YES HERE'); time.sleep(2)
            e=S('arr%d'%k)
            if e['d11']<0.6 and e.get('d0')=='0' and e.get('d5')=='0': break
        continue
    try: hdg=float(e0['d4'])
    except Exception: hdg=185.0
    cur = e0['d11']
    brg, mag = fitbearing()
    if brg is None or mag<0.02: brg = 185.0
    diff = (brg - hdg) % 360.0
    l = lidar()
    front = min([x for x in (l[15],l[0],l[1],l[2]) if x>0]+[99])
    right = min([l[i] for i in (4,5,6) if l[i]>0] or [9.9])
    left  = min([l[i] for i in (10,11,12) if l[i]>0] or [9.9])
    acted=''
    if cur>=GREEDY:
        acted='greedy'
        if front>0.32: drive(15)
        else:
            if right>=left: turn(+90)
            else: turn(-90)
            drive(10)
    elif front<=0.30:
        acted='blocked'
        consec_blocked+=1
        if consec_blocked>=3:
            d.set(-1,-1); time.sleep(2.5); d.set(0,0); time.sleep(0.3)
            consec_blocked=0
        # turn toward bearing as far as wall allows
        if diff<45 or diff>315: turn(+90 if right>=left else -90)
        elif diff<180: turn(min(85, max(40,int(diff))))
        else: turn(-min(85, max(40,int(360-diff))))
        drive(6)
    else:
        if diff<40 or diff>320:
            acted='onbearing'
            drive(15)
        elif diff<180:
            acted='right%d'%int(diff)
            turn(min(70,int(diff))); drive(8)
        else:
            acted='left%d'%int(360-diff)
            turn(-min(70,int(360-diff))); drive(8)
    if acted!='blocked':
        consec_blocked=0
    e1=S('leg%d-b'%n, act=acted, brg=None if brg is None else round(brg))
    dd=e1['d11']-e0['d11']
    try:
        dist=abs((int(float(e1['eL']))-int(float(e0['eL']))))
    except Exception:
        dist=100
    hist.append((hdg,dd,dist)); 
    if len(hist)>120: hist.pop(0)
    print('PURSUIT act=%s dd=%+.3f brg=%s mag=%s'%(acted,dd,round(brg) if brg else '-',round(mag,3)),flush=True)

d.set(0,0); time.sleep(0.5); d.close()
json.dump(log, open('/memory/ep17.json','w'))
print('DONE',flush=True)
