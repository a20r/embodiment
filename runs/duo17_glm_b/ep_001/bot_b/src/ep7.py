import sys,time,math,statistics,os
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/ep7.log','a')
SCAN=open('/memory/scan_ep7.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass

pose={'x':0.0,'y':0.0,'dist':0.0}
o0=None; hprev=None
def upd():
    global o0,hprev
    o1=R.odom()
    if o0 is None: o0=o1; return
    dl=o1[0]-o0[0]; dr=o1[1]-o0[1]
    if abs(dl)>500 or abs(dr)>500: o0=o1; return
    o0=o1
    d=(dl+dr)/2*0.00066
    pose['dist']+=abs(d)
    h=R.heading()
    if hprev is not None:
        dh=((h-hprev+180)%360)-180
        if abs(dh)>90: return
        pose['x']+=d*math.cos(math.radians(h))
        pose['y']+=d*math.sin(math.radians(h))
    hprev=h

GRID=set()
def med_lidar(n=2,tries=20):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.02)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

def txstate():
    st=R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':')[1] if ':' in tok else '?'
    return '?'

BN={'n':0}
def beacon(msg):
    BN['n']+=1
    try: R.tx('%s #%d'%(msg,BN['n']))
    except Exception: pass

def sig(n=10,dt=0.05):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    if not vs: return 0
    vs.sort()
    return vs[-2]  # near-peak (2nd largest) to catch pulses, skip single outliers

def load_state():
    try:
        import json as _j
        d=_j.load(open('/memory/pose.json'))
        pose['x']=d['x']; pose['y']=d['y']; pose['dist']=d.get('dist',0)
        GRID.update(set(map(tuple,d.get('grid',[]))))
        L('loaded pose %.1f,%.1f cells=%d'%(pose['x'],pose['y'],len(GRID)))
    except Exception as e:
        L('no saved state (%r)'%e)

def save_state():
    try:
        import json as _j
        _j.dump({'x':pose['x'],'y':pose['y'],'dist':pose['dist'],'grid':sorted(list(GRID))[-6000:]},open('/memory/pose.json','w'))
    except Exception: pass

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

def frontval():
    for _ in range(15):
        l=R.lidar()
        if len(l)==16:
            return min([x for x in (l[15],l[0],l[1]) if x>0] or [9]),l
        time.sleep(0.03)
    return 0,None

def driveguard(dur=2.4,sp=20):
    t0=time.time()
    while time.time()-t0<dur:
        f,_=frontval()
        if f<0.30: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.07)
    R.stop(); return True

def pick(l,BAD):
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    W={0:1.2,1:1.05,15:1.05,2:1.0,3:1.0,13:1.0,14:1.0,4:0.9,12:0.9,5:0.85,11:0.85,6:0.8,10:0.8,7:0.6,9:0.6,8:0.35}
    now=time.time()
    cands=[]
    hnow=R.heading()
    for k in range(16):
        op=min(b(k),b((k-1)%16),b((k+1)%16))
        ang=math.radians(hnow+k*22.5)
        px=pose['x']+1.2*math.cos(ang); py=pose['y']+1.2*math.sin(ang)
        newc=(int(px*2),int(py*2)) not in GRID
        cands.append((op*W[k]+(0.25 if newc else 0.0),k,op))
    cands.sort(reverse=True)
    for sc,k,op in cands:
        if BAD.get(k,0)<now-18: return k,op
    return cands[0][1],cands[0][2]

def blob(l):
    def b(i):
        v=l[i%16]; return v if v>0 else -1
    hits=[]
    for k in range(16):
        v=b(k); p2=b(k-2); n2=b(k+2)
        if 0.35<v<1.3 and p2>0.8 and n2>0.8 and p2>v+0.4 and n2>v+0.4:
            hits.append((k,round(v,2)))
    return hits

def unjam():
    L('UNJAM')
    R.motors(-16,-16); time.sleep(1.2)
    l=med_lidar()
    if l:
        def b(i):
            v=l[i%16]; return v if v>0 else 2.5
        best=-1;bk=8
        for k in range(16):
            op=min(b(k),b((k-1)%16),b((k+1)%16))
            if op>best: best=op;bk=k
        turnto((R.heading()+bk*22.5)%360)
    R.motors(20,20); time.sleep(1.6); R.stop()

def wander(BAD):
    l=med_lidar()
    if l is None: R.stop(); time.sleep(0.05); return
    upd()
    GRID.add((int(pose['x']*2),int(pose['y']*2)))
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    f=min(b(15),b(0),b(1))
    hits=blob(l)
    if hits:
        L('blob-note %s pos=%.1f,%.1f'%(hits,pose['x'],pose['y']))
        SCAN.write('BLOB %.2f %.2f %.1f %s\n'%(pose['x'],pose['y'],R.heading(),','.join('%.2f'%v for v in l))); SCAN.flush()
    if f<0.12:
        unjam(); return
    if f<0.18:
        R.motors(-14,-14); time.sleep(0.7); R.stop()
    k,op=pick(l,BAD)
    h=R.heading()
    if k==0: ok=driveguard(2.6,21)
    elif k in (1,15):
        bias=4 if k==1 else -4
        t0=time.time()
        ok=True
        while time.time()-t0<2.4:
            ff,_=frontval()
            if ff<0.30: R.stop(); ok=False; break
            R.motors(25,17 if bias>0 else 17); R.motors(21+bias,21-bias); time.sleep(0.07)
        R.stop()
    else:
        turnto((h+k*22.5)%360)
        ok=driveguard(2.4,21)
    if not ok: BAD[k]=time.time()

def home_step():
    base_h=R.heading()
    s0=sig()
    best=(s0,0.0,None)
    for dd in (0,90,-90,180):
        tgt=(base_h+dd)%360
        turnto(tgt)
        f,_=frontval()
        if f<0.30: continue
        ok=driveguard(2.2,20)
        s=sig()
        L('HOME probe %+d ok=%s d11=%.3f'%(dd,ok,s))
        if ok and s>best[0]: best=(s,dd,tgt)
        if not ok:
            turnto(base_h)
            driveguard(1.2,20)
    if best[2] is not None:
        turnto(best[2])
        driveguard(2.2,20)
        L('HOME commit dir %+d d11 %.3f->%.3f'%(best[1],s0,best[0]))
        return best[0]
    else:
        L('HOME all blocked')
        R.motors(-15,-15); time.sleep(0.9); R.stop()
        return s0

def main():
    load_state()
    L('EP7 start pos=%.1f,%.1f d11=%.3f tx=%s'%(pose['x'],pose['y'],sig(),txstate()))
    BAD={}
    last_be=0; last_sum=0; last_moving=0
    while True:
        try:
            st=R.status_d()
            if st.get('here')=='1':
                L('*** HERE=1 %s'%st); R.stop()
                for i in range(6):
                    beacon('B IS AT GOAL (my d3 here=1). A: COME NOW. DRIVE TOWARD RISING d11 SIGNAL. GOAL EMITS IT. WHEN YOUR here=1 WE ARE DONE.')
                    m=R.rx()
                    if m: L('RX %s'%m[:130])
                    time.sleep(0.55)
                time.sleep(1.2); continue
            if st.get('goal','0') not in ('0',''):
                L('*** GOALFLAG %s'%st)
            m=R.rx()
            if m:
                L('RX %s'%m[:130])
                if 'MOVING' in m: last_moving=time.time()
            ts=txstate()
            s=sig(3,0.04)
            now=time.time()
            if ts in ('ok','busy') and s>0.72 and now-last_moving<45:
                L('HOLD: A approaching d11=%.3f'%s)
                R.stop()
                beacon('B HOLDING FOR YOU.')
                time.sleep(0.8)
                continue
            if now-last_be>2.5:
                last_be=now
                beacon('B SEEK A+GOAL. A: KEEP TX; B HOMES ON d11 WHEN YOU SPEAK.')
            if now-last_sum>60:
                last_sum=now
                upd()
                L('SUM pos=%.1f,%.1f dist=%.1f cells=%d d11=%.3f tx=%s'%(pose['x'],pose['y'],pose['dist'],len(GRID),s,ts))
                save_state()
            if ts in ('busy','ok') and s>0.60:
                # A transmitting steadily: gradient home
                s1=home_step()
                if s1>0.82:
                    L('*** d11=%.3f HOLD+BURST ***'%s1)
                    R.stop()
                    for i in range(8):
                        beacon('B ADJACENT d11=%.2f. A LOOK AROUND.'%s1)
                        time.sleep(0.5)
            else:
                wander(BAD)
                time.sleep(0.05)
        except Exception as e:
            try: R.stop(); L('EXC %r'%e)
            except Exception: pass
            time.sleep(0.4)

if __name__=='__main__':
    main()
