import sys,time,math,statistics
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/fetch.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass

pose={'x':-3.6,'y':1.6}  # approx goal pose (ep7 frame)
o0=None
def upd():
    global o0
    o1=R.odom()
    if o0 is None: o0=o1; return
    dl=o1[0]-o0[0]; dr=o1[1]-o0[1]
    if abs(dl)>500 or abs(dr)>500: o0=o1; return
    o0=o1
    d=(dl+dr)/2*0.00066
    h=R.heading()
    pose['x']+=d*math.cos(math.radians(h)); pose['y']+=d*math.sin(math.radians(h))

def txstat():
    st=R.status()
    try: return st.split('tx=')[1].split(':',1)[1]
    except: return '?'
def beacon(msg):
    try: R.tx('B-TO-A: %s'%msg)
    except Exception: pass
def sample(n=8,dt=0.05):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0
def med_lidar():
    scans=[]
    for _ in range(25):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=2: break
        time.sleep(0.02)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]
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
    for _ in range(15):
        l=R.lidar()
        if len(l)==16:
            return min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
        time.sleep(0.03)
    return 0
def driveguard(dur=3.5,sp=21):
    t0=time.time()
    while time.time()-t0<dur:
        if front()<0.30: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.07)
    R.stop(); return True

def radiocheck():
    # TX + listen; return (status_ok_seen, rx_lines, d11)
    ok=False; rxs=[]
    for i in range(3):
        beacon('B FETCHING A. A: REPLY OR KEEP TX. I HEAR RADIO. WHERE ARE YOU?')
        t0=time.time()
        while time.time()-t0<1.2:
            m=R.rx()
            if m:
                rxs.append(m[:90]); L('RX %s'%m[:90])
            if txstat()=='ok': ok=True
            time.sleep(0.1)
    return ok,rxs,sample()

def unjam():
    L('UNJAM')
    R.motors(-16,-16); time.sleep(1.2); R.stop()
    l=med_lidar()
    if l:
        def b(i):
            v=l[i%16]; return v if v>0 else 2.5
        best=-1;bk=8
        for k in range(16):
            op=min(b(k),b((k-1)%16),b((k+1)%16))
            if op>best: best=op;bk=k
        turnto((R.heading()+bk*22.5)%360)
    R.motors(20,20); time.sleep(1.5); R.stop()

def main():
    L('FETCH start pos=%.1f,%.1f'%(pose['x'],pose['y']))
    upd()
    base=R.heading()
    ring=0
    BAD={}
    while True:
        try:
            st=R.status_d()
            if st.get('here')=='1':
                L('BACK AT GOAL')
            if st.get('goal')=='1':
                L('*** GOAL=1: A AT GOAL! RETURN! ***')
            ok,rxs,s=radiocheck()
            L('hub pos=%.1f,%.1f ok=%s rx=%d d11=%.3f'%(pose['x'],pose['y'],ok,len(rxs),s))
            if ok or rxs:
                L('*** A RADIO IN RANGE - MEET MODE ***')
                meet_mode()
                return
            if s>0.70 and txstat()=='busy':
                L('*** strong carrier d11=%.3f - MEET MODE ***'%s)
                meet_mode()
                return
            # pick direction: prefer unvisited-ish, spiral outward
            l=med_lidar()
            if l is None: continue
            def b(i):
                v=l[i%16]; return v if v>0 else 2.5
            cands=[]
            h=R.heading()
            for k in range(16):
                op=min(b(k),b((k-1)%16),b((k+1)%16))
                if k in BAD and BAD[k]>time.time()-20: continue
                ang=math.radians(h+k*22.5)
                far=math.hypot(pose['x']+3.0*math.cos(ang),pose['y']+3.0*math.sin(ang))
                sc=op+(0.4 if far>2.0 else 0.0)+(0.2 if k==0 else 0)
                cands.append((sc,k,op))
            cands.sort(reverse=True)
            k=cands[0][1]
            tgt=(h+k*22.5)%360
            if k not in (0,):
                turnto(tgt)
            moved=driveguard(3.5,21)
            upd()
            if not moved:
                BAD[k]=time.time()
                if len(BAD)>5: BAD={}
                unjam()
            if front()<0.12: unjam()
        except Exception as e:
            try: R.stop(); L('EXC %r'%e)
            except Exception: pass
            time.sleep(0.4)

def peak(n=40,dt=0.1):
    # peak of d11 over ~4s to catch A's burst
    mx=0
    for _ in range(n):
        v=R.fget('d11')
        if v>mx: mx=v
        time.sleep(dt)
    return mx

def meet_mode():
    L('FARFIELD: move away from goal, then peak-hunt A')
    # current dist from goal approx via pose
    # phase 1: push 5m in current heading direction (guarded legs)
    for leg in range(3):
        if front()<0.3: break
        driveguard(2.5,20)
        upd()
    L('farfield pushed, d11peak=%.3f'%peak())
    bestp=peak()
    hub_ang=None
    while True:
        ok,rxs,s=radiocheck()
        p=peak()
        L('farfield hub pos=%.1f,%.1f ok=%s rx=%d peak=%.3f'%(pose['x'],pose['y'],ok,len(rxs),p))
        if rxs or ok:
            L('*** A RADIO - CLOSE HUNT ***')
            close_hunt()
            return
        if p>0.82:
            L('*** PEAK %.3f CLOSE ***'%p)
            close_hunt()
            return
        # probe 3 dirs, choose highest peak
        base_h=R.heading()
        results=[]
        for dd in (0,90,-90):
            tgt=(base_h+dd)%360
            turnto(tgt)
            if front()<0.3:
                results.append((-1,dd,None)); continue
            driveguard(2.2,20)
            pp=peak()
            results.append((pp,dd,tgt))
            L('far probe %+d peak=%.3f'%(dd,pp))
            turnto(base_h)
            driveguard(1.0,20)
        results.sort(reverse=True)
        pp,dd,tgt=results[0]
        if tgt is not None and pp>bestp-0.005:
            turnto(tgt); driveguard(2.2,20)
            bestp=max(bestp,pp)
            L('far commit dir %+d peak=%.3f'%(dd,pp))
        else:
            L('far all worse (best %.3f) - rotate'%bestp)
            turnto((base_h+135)%360)

def close_hunt():
    L('CLOSE HUNT: d11 gradient with burst peaks')
    s0=peak()
    while True:
        if s0>0.88:
            L('*** ADJACENT %.3f hold ***'%s0)
            R.stop()
            for i in range(10):
                beacon('B ADJACENT. A: LOOK AROUND. FOLLOW ME TO GOAL.')
                m=R.rx()
                if m: L('RX %s'%m[:100])
                time.sleep(0.5)
            s0=peak()
            continue
        base_h=R.heading(); best=(s0,None)
        for dd in (0,90,-90,180):
            tgt=(base_h+dd)%360
            turnto(tgt)
            if front()<0.30: continue
            driveguard(2.0,20)
            pp=peak(25,0.08)
            L('close probe %+d peak=%.3f'%(dd,pp))
            if pp>best[0]: best=(pp,tgt)
            turnto(base_h); driveguard(0.8,20)
        if best[1] is not None:
            turnto(best[1]); driveguard(2.0,20)
            s0=peak()
            L('close commit peak=%.3f'%s0)
        else:
            R.motors(-15,-15); time.sleep(0.9); R.stop()
            s0=peak()

if __name__=='__main__':
    main()
