import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meet3.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass

def sample(n=10,dt=0.07):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

def turnto(tgt,sp=24,timeout=3.5):
    t0=time.time()
    while time.time()-t0<timeout:
        h=R.heading()
        err=((tgt-h+180)%360)-180
        if abs(err)<7: break
        if err>0: R.motors(sp,-sp)
        else: R.motors(-sp,sp)
        time.sleep(0.07)
    R.stop(); time.sleep(0.1)

def frontclear():
    for _ in range(12):
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            return f
        time.sleep(0.03)
    return 0.0

def driveguard(dur=2.0,sp=19):
    t0=time.time()
    while time.time()-t0<dur:
        f=frontclear()
        if f<0.28: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.07)
    R.stop(); return True

L('MEET3 hillclimb start d11=%.3f'%sample())
best=sample()
base_h=R.heading()
dirs=[0,45,-45,90,-90,135,-135,180]
di=0
fails=0
t_ann=0
while True:
    if time.time()-t_ann>12:
        t_ann=time.time()
        R.tx('B APPROACHING YOU VIA d11 HILLCLIMB. KEEP BEACON. IF SEE ME: SAY SEE.')
    h=R.heading()
    tgt=(base_h+dirs[di%8])%360
    turnto(tgt)
    f=frontclear()
    if f<0.30:
        di+=1; continue
    ok=driveguard(2.0,19)
    s1=sample()
    L('probe dir=%+d ok=%s d11=%.3f (best %.3f)'%(dirs[di%8],ok,s1,best))
    if s1>best+0.006:
        best=s1; fails=0
        L('IMPROVED d11=%.3f'%best)
    else:
        fails+=1
        di+=1
        if fails>=6:
            L('all dirs fail - reverse course')
            base_h=(base_h+180)%360
            di=0; fails=0
    if best>0.82:
        L('*** d11=%.3f > 0.82 HOLD+BURST ***'%best)
        R.stop()
        for i in range(8):
            R.tx('B HERE d11=%.2f. A: LOOK AROUND. I AM CLOSE.'%best)
            time.sleep(0.6)
        best=sample()
