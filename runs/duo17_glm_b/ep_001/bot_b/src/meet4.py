import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meet4.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
def sample(n=8,dt=0.06):
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
            return min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
        time.sleep(0.03)
    return 0.0
def driveguard(dur,sp=19):
    t0=time.time()
    while time.time()-t0<dur:
        if frontclear()<0.28: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.07)
    R.stop(); return True

PHASE='ASK'
L('MEET4 start. PHASE=ASK (hold 60s, A asked to approach)')
t0=time.time(); last_tx=0; seen=0
while PHASE=='ASK':
    if time.time()-last_tx>2.0:
        last_tx=time.time()
        R.tx('B HOLDS STILL 60s. A: DRIVE TO ME NOW. USE YOUR d11. STRAIGHT SLOW. GO!')
    m=R.rx()
    if m:
        L('RX %s'%m[:120])
        if 'MOVING' in m: seen+=1
        if 'SEE' in m and 'NOTSEE' not in m: seen+=1
    v=sample(4,0.05)
    if v>0.82: L('d11 %.3f HIGH'%v)
    if time.time()-t0>60:
        if seen>=2:
            L('A claims moving/seeing - extend ASK 60s more')
            t0=time.time(); seen=0
        else:
            PHASE='PROBE18'
L('PHASE=%s d11=%.3f hdg=%.0f'%(PHASE,sample(),R.heading()))
if PHASE=='PROBE18':
    tgt=18.0
    turnto(tgt)
    for i in range(30):
        R.tx('B TESTING BEARING 18. A KEEP BEACON.')
        ok=driveguard(1.8,19)
        v=sample()
        L('probe18 leg%d ok=%s d11=%.3f'%(i,ok,v))
        if v>0.80:
            L('*** d11 %.3f RISE >0.80 - HOLD+BURST ***'%v)
            R.stop()
            for k in range(10):
                R.tx('B ADJACENT? d11=%.2f. A LOOK AROUND.'%v)
                m=R.rx()
                if m: L('RX %s'%m[:120])
                time.sleep(0.5)
            break
        if not ok:
            L('blocked - arc right retry')
            R.motors(17,-9); time.sleep(1.0); R.stop()
            turnto(tgt)
