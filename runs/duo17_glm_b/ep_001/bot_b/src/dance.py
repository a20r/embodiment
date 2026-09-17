import sys,time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/dance.log','a')
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
def drive(dur=2.0,sp=20):
    t0=time.time()
    while time.time()-t0<dur:
        if here() or goal():
            R.stop(); return 'flag'
        if front()<0.30:
            R.stop(); return 'blocked'
        R.tx('B-TO-A: A: FOLLOW B. DRIVE UNTIL YOUR here=1 FIRES THEN STOP.')
        R.motors(sp,sp); time.sleep(0.1)
    R.stop(); return 'ok'
def reacquire():
    L('reacquire goal zone')
    for i in range(40):
        if here(): return True
        if goal(): return True
        f=front()
        if f<0.30:
            right=None;left=None
            for _ in range(10):
                l=R.lidar()
                if len(l)==16:
                    right=min([x for x in (l[10],l[11],l[12]) if x>0] or [9])
                    left=min([x for x in (l[4],l[5],l[6]) if x>0] or [9])
                    break
                time.sleep(0.03)
            turnto(R.heading()+(70 if (right or 0)>(left or 0) else -70))
        drive(1.4,20)
    return here() or goal()
def main():
    L('DANCE start here=%s goal=%s'%(here(),goal()))
    # phase 1: wait for A close
    peak=0
    t0=time.time()
    while time.time()-t0<600:
        v=R.fget('d11')
        if v>peak: peak=v; L('PEAK %.3f'%peak)
        R.tx('B-TO-A: A: DRIVE TOWARD d11 RISING UNTIL YOUR here=1 FIRES. THAT IS THE GOAL. THEN STOP THERE. B IS THERE NOW.')
        m=R.rx()
        if m: L('RX %s'%m[:110])
        if goal():
            L('*** GOAL=1 while holding: A IN ZONE ***')
            break
        if peak>0.80:
            L('*** A CLOSE (peak %.3f) - EXIT ZONE ***'%peak)
            break
        time.sleep(0.25)
    else:
        L('timeout waiting; stay at goal')
        return
    if here() and goal():
        L('*** BOTH FLAGS 1 ALREADY - FINAL HOLD ***')
        t0=time.time()
        while time.time()-t0<1800:
            R.tx('B-TO-A: B AT GOAL. A AT GOAL. DONE!')
            if 'here=0' in R.status():
                reacquire()
            time.sleep(0.5)
        return
    # phase 2: exit zone, keep flooding
    h0=R.heading()
    turnto((h0+180)%360)
    for i in range(3):
        r=drive(1.2,22)
        if r=='flag': break
    L('exited zone: here=%s goal=%s'%(here(),goal()))
    # phase 3: let A enter, then re-enter
    t0=time.time()
    a_in=False
    while time.time()-t0<300:
        R.tx('B-TO-A: A: ENTER THE ZONE WHERE YOUR here=1 FIRES. STOP THERE.')
        if goal() and not here():
            a_in=True
            L('*** A IN ZONE (goal=1, here=0) - RUSH IN ***')
            break
        v=R.fget('d11')
        if v>0.88:
            L('*** A ADJACENT (%.3f) - LEAD IN ***'%v)
            break
        time.sleep(0.3)
    turnto(h0)
    for i in range(6):
        r=drive(1.4,22)
        st=R.status()
        L('rush: %s here=%s goal=%s'%(r,'here=1' in st,'goal=1' in st))
        if here() and goal():
            L('*** BOTH FLAGS 1 - HOLD POSITION ***')
            break
        if here() and not goal():
            R.stop()
            t1=time.time()
            while time.time()-t1<120:
                R.tx('B-TO-A: A COME INTO ZONE. B IS INSIDE. COME!')
                if goal():
                    L('*** GOAL=1 - BOTH IN! ***')
                    break
                time.sleep(0.4)
            break
    # final hold
    t0=time.time()
    while time.time()-t0<900:
        R.tx('B-TO-A: B AT GOAL. A COME. WE FINISH TOGETHER.')
        st=R.status()
        if 'here=0' in st:
            L('lost zone - reacquire')
            reacquire()
        if 'goal=1' in st and 'here=1' in st:
            L('*** DONE? both flags 1 ***')
        m=R.rx()
        if m: L('RX %s'%m[:100])
        time.sleep(0.3)
main()
