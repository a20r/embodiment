import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meet5.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
def sample(n=9,dt=0.06):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else -1
def turnto(tgt,sp=24,timeout=3):
    t0=time.time()
    while time.time()-t0<timeout:
        h=R.heading()
        err=((tgt-h+180)%360)-180
        if abs(err)<7: break
        if err>0: R.motors(sp,-sp)
        else: R.motors(-sp,sp)
        time.sleep(0.06)
    R.stop(); time.sleep(0.1)
def front():
    for _ in range(15):
        l=R.lidar()
        if len(l)==16:
            return min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
        time.sleep(0.03)
    return 0
def drive(dur=2.2,sp=20):
    t0=time.time()
    while time.time()-t0<dur:
        if front()<0.3: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.07)
    R.stop(); return True

L('MEET5 gradient ascent start d11=%.3f'%sample())
base_h=R.heading()
best=sample()
t_ann=0; stuck_n=0
while True:
    if time.time()-t_ann>6:
        t_ann=time.time()
        R.tx('B HOMING ON YOUR CARRIER (d11 rising). KEEP TX CONTINUOUS. DO NOT MOVE. #%d'%(t_ann%1000))
    res=[]
    for dd in (0,90,-90,180):
        tgt=(base_h+dd)%360
        turnto(tgt)
        if front()<0.30:
            res.append((None,dd)); continue
        ok=drive(2.3,20)
        s=sample()
        res.append((s if ok else None,dd))
        L('probe %+d ok=%s d11=%.3f'%(dd,ok,s))
        if best is not None and s>best+0.004 and ok:
            best=s
            L('NEW BEST %.3f dir %+d'%(best,dd))
            break  # commit, keep going this way next loop
    else:
        good=[(s,dd) for s,dd in res if s is not None]
        if good:
            good.sort(reverse=True)
            bs,bd=good[0]
            if bs>best+0.004:
                best=bs
                L('BEST %.3f dir %+d'%())
        # no improvement: rotate search
        stuck_n+=1
        base_h=(base_h+45)%360
        L('no improve; rotate base -> %.0f (stuck %d)'%(base_h,stuck_n))
        if stuck_n>6:
            L('stuck>6: unjam reverse')
            R.motors(-16,-16); time.sleep(1.0); R.stop()
            stuck_n=0
    if best>0.82:
        L('*** d11=%.3f HOLD+BURST ***'%best)
        R.stop()
        for i in range(10):
            R.tx('B VERY CLOSE (d11=%.2f). A: I AM ADJACENT. LOOK.'%best)
            m=R.rx()
            if m: L('RX %s'%m[:120])
            time.sleep(0.5)
        best=sample()
