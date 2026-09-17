import sys,time,math,statistics,random
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/ep4.log','a')
SCAN=open('/memory/scan_ep4.log','a')
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
    try: R.tx('B-TO-A: %s #%d'%(msg,BN['n']))
    except Exception: pass

def sig(n=4,dt=0.05):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

def ang_of(k): return k*22.5

def pick(l):
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    W={0:1.0,1:1.0,2:1.0,3:1.0,4:0.9,13:1.0,14:1.0,15:1.0,5:0.8,6:0.75,12:0.8,11:0.75,7:0.55,10:0.55,8:0.3,9:0.3}
    best=-1; bk=0
    for k in range(16):
        op=min(b(k),b(k-1),b(k+1))
        sc=op*W.get(k,0.5)
        if sc>best: best=sc; bk=k
    return bk,best

def turnto(tgt,sp=24,timeout=4):
    t0=time.time()
    while time.time()-t0<timeout:
        h=R.heading()
        err=((tgt-h+180)%360)-180
        if abs(err)<7: break
        if err>0: R.motors(sp,-sp)
        else: R.motors(-sp,sp)
        time.sleep(0.07)
    R.stop(); time.sleep(0.12)

def driveguard(dur=2.6,sp=20):
    t0=time.time()
    while time.time()-t0<dur:
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            if f<0.28: R.stop(); return False
        R.motors(sp,sp); time.sleep(0.06)
    R.stop(); return True

def pick(l,BAD):
    def b(i):
        v=l[i%16]; return v if v>0 else 2.5
    W={0:1.2,1:1.05,15:1.05,2:1.0,3:1.0,13:1.0,14:1.0,4:0.9,12:0.9,5:0.85,11:0.85,6:0.8,10:0.8,7:0.6,9:0.6,8:0.35}
    now=time.time()
    cands=[]
    for k in range(16):
        op=min(b(k),b((k-1)%16),b((k+1)%16))
        cands.append((op*W[k],k,op))
    cands.sort(reverse=True)
    for sc,k,op in cands:
        if BAD.get(k,0)<now-18: return k,op
    return cands[0][1],cands[0][2]

def driveguard_arc(dur=2.8,bias=4,sp=21):
    t0=time.time()
    while time.time()-t0<dur:
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            if f<0.28: R.stop(); return False
        R.motors(sp+bias,sp-bias); time.sleep(0.06)
    R.stop(); return True

def wander():
    BAD={}
    n=0
    while True:
        n+=1
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd()
        GRID.add((int(pose['x']*2),int(pose['y']*2)))
        def b(i):
            v=l[i%16]; return v if v>0 else 2.5
        f=min(b(15),b(0),b(1))
        if f<0.18:
            R.motors(-14,-14); time.sleep(0.7); R.stop()
        k,op=pick(l,BAD)
        h=R.heading()
        if k==0:
            ok=driveguard(3.2,21)
        elif k in (1,15):
            bias=4 if k==1 else -4
            ok=driveguard_arc(2.8,bias)
        else:
            tgt=(h+k*22.5)%360
            turnto(tgt)
            ok=driveguard(2.8,21)
        if not ok:
            BAD[k]=time.time()
        if n%8==0:
            try:
                import json as _j
                _j.dump({'grid':sorted(list(GRID))[-4000:],'x':pose['x'],'y':pose['y'],'dist':pose['dist']},open('/memory/vis.json','w'))
            except Exception: pass
        if n%2==0:
            s=sig(2,0.04)
            L('STEP pos=%.1f,%.1f k=%d op=%.2f ok=%s d11=%.3f tx=%s cells=%d'%(pose['x'],pose['y'],k,op,ok,s,txstate(),len(GRID)))
            SCAN.write('%.2f %.2f %s\n'%(pose['x'],pose['y'],','.join('%.2f'%v for v in l))); SCAN.flush()
def main():
    L('EP4 start d11=%.3f tx=%s'%(sig(),txstate()))
    last_be=0; last_sum=0
    while True:
        try:
            st=R.status_d()
            if st.get('here')=='1':
                L('*** HERE=1 %s'%st); R.stop()
                for i in range(10):
                    beacon('AT GOAL here=1. A COME NOW')
                    time.sleep(0.5)
                time.sleep(2); continue
            if st.get('goal','0') not in ('0',''):
                L('*** GOALFLAG %s'%st)
            m=R.rx()
            if m: L('RX!! %s'%m[:150])
            ts=txstate()
            if ts in ('ok','busy'):
                L('*** PEER RANGE tx=%s - stop+burst'%ts)
                R.stop()
                for i in range(6):
                    beacon('HEAR YOU. STOPPED. SEND POS/HDG')
                    m=R.rx()
                    if m: L('RX!! %s'%m[:150])
                    time.sleep(0.6)
            if time.time()-last_be>2.4:
                last_be=time.time()
                beacon('B alive pos=%.1f,%.1f seek A+GOAL'%(pose['x'],pose['y']))
            if time.time()-last_sum>45:
                last_sum=time.time()
                upd()
                L('SUM pos=%.1f,%.1f dist=%.1f cells=%d d11=%.3f tx=%s'%(pose['x'],pose['y'],pose['dist'],len(GRID),sig(3,0.04),txstate()))
            wander()
        except Exception as e:
            try: R.stop(); L('EXC %r'%e)
            except Exception: pass
            time.sleep(0.4)

if __name__=='__main__':
    main()
