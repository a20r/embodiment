import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore13.log','a')
MAP=open('/memory/map13.log','a')
SIG=open('/memory/sig13.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=['B ALIVE. SEEKING GOAL.','B: HOME TO MY SIGNAL IF YOU REVIVE.','B: BEEP.']
    try:
        R.tx(msgs[int(time.time())%3]); MYTX['t']=time.time()
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
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
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
    # weight: openness * direction preference toward front
    best=None; bs=-1
    for k in range(16):
        v=l[k]
        if v<=0: continue
        nb=min([x for x in (l[(k-1)%16],l[k],l[(k+1)%16]) if x>0] or [0])
        if nb<0.33: continue
        off=min(k,16-k)
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

def main():
    L('EXPLORE13 open-pursuit start')
    last_rep=0; SIGMIN=9.0; last_cov=0; last_cov_t=time.time(); last_d0log=0
    last_pos=(0,0); last_pos_t=time.time()
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        d0=R.fget('d0')
        if d0!=0:
            if time.time()-last_d0log>5:
                L('D0 contact %.1f at %.2f,%.2f'%(d0,pose['x'],pose['y']))
                last_d0log=time.time()
            R.motors(16,16); time.sleep(0.22); R.stop(); upd()
            continue
        m=R.rx()
        if m: L('RX!! %s'%m.replace('\n',' | ')[:120])
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd(); mark(); map_log(l)
        s=sig_med()
        SIG.write('%.2f %.2f %.3f %s %.2f\n'%(pose['x'],pose['y'],s,txstate(),pose['dist'])); SIG.flush()
        if s<SIGMIN: SIGMIN=s
        if s>SIGMIN+0.05:
            L('d11 RISE %.3f (min %.3f) at %.2f,%.2f'%(s,SIGMIN,pose['x'],pose['y']))
            SIGMIN=s
        if s>0.62 and time.time()-MYTX['t']>2.0:
            L('HOTSPOT %.3f'%s); R.stop(); time.sleep(1.2)
            s2=sig_med(20)
            if s2>0.62:
                for i in range(6):
                    try: R.tx('B: I SEE SIGNAL. REPLY!')
                    except Exception: pass
                    time.sleep(0.25)
                continue
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        rear=min([x for x in (l[7],l[8],l[9]) if x>0] or [0])
        tight=min([x for x in l if x>0] or [0])
        mode=''
        if front>=0.35:
            k=best_beam(l)
            if k is None: k=0
            # steer toward beam bearing with gentle motion
            ang=22.5*k if k<=8 else 22.5*(k-16)
            err=ang
            corr=max(-12,min(12, -err*0.55))
            R.motors(19-corr, 19+corr)
            mode='pursue%d'%k
        elif rear>=0.35:
            k=best_beam(l)
            ang=22.5*k if k<=8 else 22.5*(k-16)
            if k is not None and 5<=k<=11:
                # opening behind: back up while arcing toward it
                if ang>0: R.motors(-13,-25)
                else: R.motors(-25,-13)
                time.sleep(0.7)
            else:
                R.motors(-19,-19); time.sleep(0.7)
            R.stop(); upd(); mode='backarc'
        elif tight>0.42:
            left=avgc(l,range(1,7)); right=avgc(l,range(10,16))
            if left>=right: R.motors(-20,20)
            else: R.motors(20,-20)
            time.sleep(0.5); R.stop(); upd(); mode='rot'
        else:
            R.motors(-16,-16); time.sleep(0.5); R.stop(); upd(); mode='nudge'
        if time.time()-MYTX['t']>30.0: beacon_tick()
        # watchdogs
        if len(GRID)>last_cov:
            last_cov=len(GRID); last_cov_t=time.time()
        nowp=(round(pose['x'],1),round(pose['y'],1))
        if nowp!=last_pos:
            last_pos=nowp; last_pos_t=time.time()
        if time.time()-last_cov_t>50 or time.time()-last_pos_t>35:
            L('STALL: cov=%d pos=%s — committed push'%(len(GRID),nowp))
            k=max(range(16), key=lambda i: l[i] if l[i]>0 else -1)
            tgt=(pose['h']+22.5*k)%360
            for _ in range(14):
                err=((tgt-R.heading()+180)%360)-180
                if abs(err)>4:
                    if err>0: R.motors(22,-22)
                    else: R.motors(-22,22)
                else:
                    R.motors(19,19)
                time.sleep(0.28)
                R.stop(); time.sleep(0.04)
                l2=med_lidar()
                if l2:
                    f2=min([x for x in (l2[15],l2[0],l2[1]) if x>0] or [0])
                    if f2<0.22: break
            upd(); last_cov_t=time.time(); last_pos_t=time.time()
        if time.time()-last_rep>40:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d dist=%.2f tx=%s d11=%.3f lidar=%s'%(
                pose['x'],pose['y'],pose['h'],mode,len(GRID),pose['dist'],txstate(),s,','.join('%.1f'%v for v in l)))
        time.sleep(0.04)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE13 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid13.json','w'))
