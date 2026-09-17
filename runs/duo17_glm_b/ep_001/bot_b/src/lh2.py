import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/lh2.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
def last(v):
    p=[x for x in v.split('\n') if x.strip()]
    return p[-1] if p else ''
def here():
    d3=R.status()
    return 'here=1' in d3
def med(n=30,dt=0.06):
    import statistics
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0
def peak(n=12,dt=0.06):
    mx=0
    for _ in range(n):
        v=R.fget('d11')
        if v>mx: mx=v
        time.sleep(dt)
    return mx
def turnto(tgt,sp=24,timeout=3):
    t0=time.time()
    while time.time()-t0<timeout:
        h=R.heading()
        err=((tgt-h+180)%360)-180
        if abs(err)<7: break
        if err>0: R.motors(sp,-sp)
        else: R.motors(-sp,sp)
        time.sleep(0.06)
    R.stop(); time.sleep(0.08)
def front():
    for _ in range(12):
        l=R.lidar()
        if len(l)==16:
            return min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
        time.sleep(0.03)
    return 0
L('LH2g start tx=%s d11=%.3f here=%s'%(R.status()[-10:],R.fget('d11'),here()))
found=False
hub0=med()
L('hub0 med=%.3f'%hub0)
while not found:
    R.tx('B-TO-A: A COMING GOOD! B DRIVING TO GOAL. FOLLOW MY CARRIER. WE FINISH TOGETHER.')
    if here():
        found=True; break
    base_h=R.heading()
    results=[]
    for dd in (0,90,-90):
        tgt=(base_h+dd)%360
        turnto(tgt)
        if front()<0.30:
            continue
        t0=time.time()
        moved=False
        while time.time()-t0<2.0:
            if here():
                R.stop(); found=True; break
            if front()<0.30: break
            R.tx('B-TO-A: FOLLOW ME TO GOAL.')
            R.motors(21,21); time.sleep(0.1)
            moved=True
        R.stop()
        if found: break
        m0=med(20)
        results.append((m0,dd,tgt,moved))
        L('probe %+d med=%.3f moved=%s'%(dd,m0,moved))
        if not moved:
            turnto(base_h); continue
        # return to hub for fair comparison
        turnto((base_h+dd+180)%360 if False else (tgt+180)%360)
        t0=time.time()
        while time.time()-t0<2.0:
            if here():
                R.stop(); found=True; break
            if front()<0.30: break
            R.motors(21,21); time.sleep(0.1)
        R.stop()
        if found: break
    if found: break
    results=[r for r in results if r[3]]
    if results:
        results.sort(reverse=True)
        if results[0][0]>hub0+0.004:
            tgt=results[0][2]
            turnto(tgt)
            t0=time.time()
            while time.time()-t0<2.2:
                if here():
                    R.stop(); found=True; break
                if front()<0.30: break
                R.motors(21,21); time.sleep(0.1)
            R.stop()
            hub0=med()
            L('commit dir %+d hub=%.3f'%(results[0][1],hub0))
            continue
    # no improvement: unjam turn
    R.motors(-14,-14); time.sleep(0.5)
    turnto((base_h+120)%360)
    hub0=med()
    L('rotate hub=%.3f'%hub0)
L('*** HERE=1 AT GOAL. HOLD+FLOOD FOR A. ***')
t0=time.time()
while True:
    R.tx('B-TO-A: B AT GOAL (here=1). A: COME TO MY CARRIER. WE FINISH. COME.')
    v=R.fget('d11')
    if v>0.80: L('d11=%.3f A NEAR'%v)
    m=R.rx()
    if m: L('RX %s'%m[:110])
    time.sleep(0.15)
