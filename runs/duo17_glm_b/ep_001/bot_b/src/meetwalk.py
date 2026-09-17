import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meetwalk.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
def here(): return 'here=1' in R.status()
def goal(): return 'goal=1' in R.status()
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
def drive(dur=2.0,sp=21,flood=True):
    t0=time.time()
    while time.time()-t0<dur:
        if here():
            R.stop(); return 'goal'
        if front()<0.30:
            R.stop(); return 'blocked'
        if flood: R.tx('B-TO-A: B COMING TO MEET YOU. KEEP DRIVING TO B. d11 RISING GOOD.')
        R.motors(sp,sp); time.sleep(0.1)
    R.stop(); return 'ok'
def peak(n=25,dt=0.08):
    mx=0
    for _ in range(n):
        v=R.fget('d11')
        if v>mx: mx=v
        time.sleep(dt)
    return mx
def listen(n=12,dt=0.15):
    rxs=[];ok=False
    for _ in range(n):
        m=R.rx()
        if m: rxs.append(m[:90])
        if 'ok' in R.status(): ok=True
        time.sleep(dt)
    return rxs,ok

def main():
    L('MEETWALK start here=%s'%here())
    h0=R.heading()
    dirs=[0,90,180,-90]
    sweep=0
    while sweep<2:
        for dd in dirs:
            tgt=(h0+dd+sweep*45)%360
            turnto(tgt)
            for leg in range(2):
                r=drive(2.0,21)
                if r=='goal':
                    L('accidentally at goal'); 
                if r=='blocked':
                    # arc around
                    R.motors(20,-12); time.sleep(0.9); R.stop()
                    turnto(tgt)
            rxs,ok=listen()
            p=peak()
            L('dir %+d sweep%d rxs=%d ok=%s peak=%.3f here=%s'%(dd,sweep,len(rxs),ok,p,here()))
            for m in rxs: L('  RX %s'%m)
            if ok or rxs or p>0.75:
                L('*** SIGN OF A - PURSUE dir %+d ***'%(dd,))
                return
            # walk back to goal
            turnto((tgt+180)%360)
            for leg in range(2):
                r=drive(2.0,21,flood=False)
                if r=='goal':
                    L('back at goal mid-sweep')
                    break
                if r=='blocked':
                    R.motors(20,-12); time.sleep(0.9); R.stop()
                    turnto((tgt+180)%360)
        sweep+=1
    L('no signs after %d sweeps - return to goal and hold'%sweep)
    # return to goal: drive until here=1
    for i in range(30):
        if here(): break
        f=front()
        if f<0.30:
            R.motors(20,-12); time.sleep(0.9); R.stop()
        drive(1.5,21,flood=False)
    L('final here=%s'%here())
main()
