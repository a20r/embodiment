import sys,time,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meetfar.log','a')
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
        if flood: R.tx('B-TO-A: A STAY PUT. B COMING TO YOU. THEN WE GO TO GOAL TOGETHER.')
        R.motors(sp,sp); time.sleep(0.1)
    R.stop(); return 'ok'
def peak(n=25,dt=0.08):
    mx=0
    for _ in range(n):
        v=R.fget('d11')
        if v>mx: mx=v
        time.sleep(dt)
    return mx
def listen(n=14,dt=0.15):
    rxs=[];ok=False
    for _ in range(n):
        m=R.rx()
        if m: rxs.append(m[:90])
        if 'ok' in R.status(): ok=True
        time.sleep(dt)
    return rxs,ok

def main():
    L('MEETFAR start here=%s'%here())
    h0=R.heading()
    best=(0,None,None)
    for sweep in range(3):
        for dd in (0,45,90,135,180,-135,-90,-45):
            tgt=(h0+dd)%360
            turnto(tgt)
            legs=0
            for leg in range(3):
                r=drive(2.2,21)
                if r=='goal':
                    L('at goal mid-leg')
                if r=='blocked':
                    R.motors(20,-12); time.sleep(0.9); R.stop()
                    turnto(tgt)
                else: legs+=1
            rxs,ok=listen()
            p=peak()
            L('dir %+d legs=%d rxs=%d ok=%s peak=%.3f'%(dd,legs,len(rxs),ok,p))
            for m in rxs: L('  RX %s'%m)
            if p>best[0]: best=(p,dd,legs)
            if ok or len(rxs)>2 or p>0.78:
                L('*** STRONG SIGN dir %+d (peak %.3f ok=%s) - CLOSE PURSUIT ***'%(dd,p,ok))
                close_pursuit(tgt,legs)
                return
            # walk back
            turnto((tgt+180)%360)
            for leg in range(3):
                r=drive(2.2,21,flood=False)
                if r=='goal':
                    break
                if r=='blocked':
                    R.motors(20,-12); time.sleep(0.9); R.stop()
                    turnto((tgt+180)%360)
        L('sweep %d done best=%s'%(sweep,best))
    L('no pursuit trigger after 3 sweeps - returning to goal hold')
    for i in range(40):
        if here(): break
        if front()<0.30:
            R.motors(20,-12); time.sleep(0.9); R.stop()
        drive(1.5,21,flood=False)
    L('final here=%s'%here())

def close_pursuit(tgt,legs):
    L('CLOSE PURSUIT from dir %+d'%tgt)
    # continue in this direction watching peak; gradient at forks
    bestp=peak()
    for i in range(25):
        r=drive(2.2,21)
        p=peak()
        L('pursuit leg%d r=%s peak=%.3f'%(i,r,p))
        rxs,ok=listen(8)
        if rxs or ok:
            L('*** A RADIO VERY CLOSE (rx=%d ok=%s) ***'%(len(rxs),ok))
        if p>0.86:
            L('*** ADJACENT %.3f - LOOK FOR A ***'%p)
            R.stop()
            for k in range(12):
                R.tx('B-TO-A: B IS NEXT TO YOU. STOP. LOOK. THEN FOLLOW ME TO GOAL.')
                m=R.rx()
                if m: L('RX %s'%m[:100])
                time.sleep(0.5)
            lead_home()
            return
        if r=='blocked':
            # choose fork by burst peak: probe left/right
            base=R.heading()
            bl=0; br=0
            turnto((base+70)%360)
            bl=peak(18,0.08)
            turnto((base-70)%360)
            br=peak(18,0.08)
            tgt2=(base+(70 if bl>=br else -70))%360
            turnto(tgt2)
            L('fork L=%.3f R=%.3f -> %d'%(bl,br,70 if bl>=br else -70))
        if r=='goal':
            L('reached goal during pursuit?!')
            return
    L('pursuit ended - lead home anyway')
    lead_home()

def lead_home():
    L('LEAD HOME: drive until here=1, flood FOLLOW ME')
    for i in range(60):
        if here():
            L('*** AT GOAL - A SHOULD FOLLOW ***')
            R.stop()
            t0=time.time()
            while time.time()-t0<900:
                R.tx('B-TO-A: A: COME INTO THE ZONE. DRIVE UNTIL YOUR here=1. THEN STOP. WE ARE DONE.')
                st=R.status()
                if 'goal=1' in st and 'here=1' in st:
                    L('*** BOTH FLAGS 1 ***')
                if 'here=0' in st:
                    L('lost zone - reacquire')
                    break
                m=R.rx()
                if m: L('RX %s'%m[:100])
                time.sleep(0.35)
            return
        if front()<0.30:
            R.motors(20,-12); time.sleep(0.9); R.stop()
        drive(1.5,21)
    L('lead_home failed to reacquire')

main()
