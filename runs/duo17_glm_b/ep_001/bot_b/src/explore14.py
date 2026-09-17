import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore14.log','a')
MAP=open('/memory/map14.log','a')
SIG=open('/memory/sig14.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    try:
        R.tx('B: SEEKING GOAL. REPLY IF ALIVE.'); MYTX['t']=time.time()
    except Exception: pass

def med_lidar(n=2,tries=25):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.04)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

def sig_med(n=10,dt=0.08):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

def txstate():
    st=R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':')[1] if ':' in tok else '?'
    return '?'

o0=None
pose={'x':0.0,'y':0.0,'h':0.0,'dist':0.0}
def upd():
    global o0
    o1=R.odom()
    if o0 is None:
        o0=o1; return
    dl=o1[0]-o0[0]; dr=o1[1]-o0[1]
    if abs(dl)>400 or abs(dr)>400:
        return  # glitch, skip
    d=(dl+dr)/2*0.00066
    pose['dist']+=abs(d)
    pose['h']=R.heading()
    pose['x']+=d*math.cos(math.radians(pose['h']))
    pose['y']+=d*math.sin(math.radians(pose['h']))
    o0=o1

GRID={}
def mark():
    k=(int(pose['x']*3),int(pose['y']*3)); GRID[k]=GRID.get(k,0)+1

def avgc(l,idxs):
    v=[min(l[i],1.5) for i in idxs if l[i]>0]
    return sum(v)/len(v) if v else 1.5

def best_beam(l):
    best=None; bs=-1
    for k in range(16):
        v=l[k]
        if v<=0: continue
        nb=min([x for x in (l[(k-1)%16],l[k],l[(k+1)%16]) if x>0] or [0])
        if nb<0.33: continue
        w=[1.0,0.95,0.9,0.85,0.8,0.75,0.7,0.65,0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95][k]
        sc=min(v,1.8)*w
        if sc>bs: bs=sc; best=k
    return best

def map_log(l):
    h=math.radians(pose['h'])
    pts=[]
    for k in range(16):
        d=l[k]
        if d<=0: continue
        ang=h-math.radians(22.5*k)
        pts.append('%.2f,%.2f'%(pose['x']+d*math.cos(ang),pose['y']+d*math.sin(ang)))
    MAP.write('%.2f %.2f %.1f | %s\n'%(pose['x'],pose['y'],pose['h'],' '.join(pts))); MAP.flush()

# charge ramp detector
RAMP={'vals':[], 'last_up':0}
def charge_check(s):
    RAMP['vals'].append((time.time(),s))
    cut=time.time()-45
    RAMP['vals']=[(t,v) for t,v in RAMP['vals'] if t>cut]
    vs=[v for _,v in RAMP['vals']]
    if len(vs)>=6 and vs[-1]>vs[0]+0.05 and vs[-1]>max(vs[:-1]):
        L('*** CHARGE RAMP? %.3f over 45s ***'%vs[-1])
        return True
    return False

def charge_hunt():
    # aggressive gradient pursuit on d11
    L('CHARGE HUNT start s=%.3f'%sig_med())
    h0=R.heading()
    best=(sig_med(),0)
    for dg in (0,45,-45,90,-90,135,-135,180):
        tgt=(h0+dg)%360
        t0=time.time()
        while time.time()-t0<3:
            err=((tgt-R.heading()+180)%360)-180
            if abs(err)<5: break
            if err>0: R.motors(22,-22)
            else: R.motors(-22,22)
            time.sleep(0.08)
        R.stop()
        R.motors(19,19); time.sleep(1.0); R.stop()
        sv=sig_med(12)
        L('  dir %+d -> %.3f'%(dg,sv))
        if sv>best[0]: best=(sv,dg)
    L('CHARGE HUNT best %+d %.3f'%(best[1],best[0]))
    # commit 8s toward best
    tgt=(h0+best[1])%360
    t0=time.time()
    while time.time()-t0<10:
        err=((tgt-R.heading()+180)%360)-180
        if abs(err)<5:
            R.motors(19,19)
        else:
            if err>0: R.motors(20,-20)
            else: R.motors(-20,20)
        time.sleep(0.15)
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            if f<0.25:
                R.stop(); break
    R.stop(); upd()

HUNT={'on':False,'peak':0.0}
def charge_hunt2():
    s0=sig_med()
    h0=R.heading()
    best=(s0,0)
    for dg in (0,40,-40,80,-80,150,-150):
        tgt=(h0+dg)%360
        t0=time.time()
        while time.time()-t0<3:
            err=((tgt-R.heading()+180)%360)-180
            if abs(err)<5: break
            if err>0: R.motors(22,-22)
            else: R.motors(-22,22)
            time.sleep(0.08)
        R.stop()
        R.motors(19,19); time.sleep(1.0); R.stop()
        sv=sig_med(12)
        L('  dir %+d -> %.3f'%(dg,sv))
        if sv>best[0]: best=(sv,dg)
    L('HUNT2 best %+d %.3f (peak %.3f)'%(best[1],best[0],HUNT['peak']))
    tgt=(h0+best[1])%360
    t0=time.time()
    while time.time()-t0<9:
        err=((tgt-R.heading()+180)%360)-180
        if abs(err)<5: R.motors(19,19)
        else:
            if err>0: R.motors(20,-20)
            else: R.motors(-20,20)
        time.sleep(0.15)
        l=R.lidar()
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
            if f<0.25:
                R.stop(); break
    R.stop(); upd()
    s1=sig_med()
    if s1>HUNT['peak']:
        HUNT['peak']=s1; L('PEAK %.3f'%s1)

def main():
    L('EXPLORE14b huntlock start d11=%.3f'%sig_med())
    last_rep=0; last_cov=0; last_cov_t=time.time(); last_d0log=0
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        m=R.rx()
        if m: L('RX!! %s'%m.replace('\n',' | ')[:120])
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd(); mark(); map_log(l)
        s=sig_med()
        SIG.write('%.2f %.2f %.3f %s %.2f\n'%(pose['x'],pose['y'],s,txstate(),pose['dist'])); SIG.flush()
        if s>HUNT['peak']: HUNT['peak']=s
        if s>0.40: HUNT['on']=True
        if HUNT['on'] and (s<0.34 and s<HUNT['peak']-0.08):
            HUNT['on']=False; L('HUNT OFF s=%.3f peak=%.3f'%(s,HUNT['peak']))
        if HUNT['on']:
            charge_hunt2()
            st2=R.status_d()
            if st2.get('here')=='1' or st2.get('goal','0') not in ('0',''):
                L('*** FLAG DURING HUNT *** %s'%json.dumps(st2))
            m2=R.rx()
            if m2: L('RX!! %s'%m2[:100])
            if time.time()-MYTX['t']>15.0: beacon_tick()
            continue
        d0=R.fget('d0')
        if d0!=0:
            if time.time()-last_d0log>5:
                L('D0 contact %.1f'%d0); last_d0log=time.time()
            R.motors(16,16); time.sleep(0.22); R.stop(); upd()
            continue
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        rear=min([x for x in (l[7],l[8],l[9]) if x>0] or [0])
        tight=min([x for x in l if x>0] or [0])
        if front>=0.35:
            k=best_beam(l) or 0
            ang=22.5*k if k<=8 else 22.5*(k-16)
            corr=max(-12,min(12, -ang*0.55))
            R.motors(19-corr, 19+corr)
            mode='pursue%d'%k
        elif rear>=0.35:
            R.motors(-19,-19); time.sleep(0.7); R.stop(); upd(); mode='back'
        elif tight>0.42:
            left=avgc(l,range(1,7)); right=avgc(l,range(10,16))
            if left>=right: R.motors(-20,20)
            else: R.motors(20,-20)
            time.sleep(0.5); R.stop(); upd(); mode='rot'
        else:
            R.motors(-16,-16); time.sleep(0.5); R.stop(); upd(); mode='nudge'
        if time.time()-MYTX['t']>25.0: beacon_tick()
        if len(GRID)>last_cov:
            last_cov=len(GRID); last_cov_t=time.time()
        if time.time()-last_cov_t>55:
            L('STALL cov=%d — push'%len(GRID))
            k=max(range(16), key=lambda i: l[i] if l[i]>0 else -1)
            tgt=(pose['h']+22.5*k)%360
            for _ in range(12):
                err=((tgt-R.heading()+180)%360)-180
                if abs(err)>4:
                    if err>0: R.motors(22,-22)
                    else: R.motors(-22,22)
                else: R.motors(19,19)
                time.sleep(0.26); R.stop(); time.sleep(0.04)
                l2=med_lidar()
                if l2 and min([x for x in (l2[15],l2[0],l2[1]) if x>0] or [9])<0.22: break
            upd(); last_cov_t=time.time()
        if time.time()-last_rep>40:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d dist=%.2f tx=%s d11=%.3f lidar=%s'%(
                pose['x'],pose['y'],pose['h'],mode,len(GRID),pose['dist'],txstate(),s,','.join('%.1f'%v for v in l)))
        time.sleep(0.04)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE14 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid14.json','w'))
