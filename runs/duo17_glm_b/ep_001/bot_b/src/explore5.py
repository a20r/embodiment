import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore5.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=['B-TO-A: I AM WALL-FOLLOWING, RED CARPET TOUR. KEEP TXING.',
          'B-TO-A: IF YOU ARE STUCK, SEND: STUCK. I WILL COME.',
          'B-TO-A: GOAL? SEND GOAL. I LISTEN BETWEEN PINGS.']
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

def txstate():
    st=R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':')[1] if ':' in tok else '?'
    return '?'

o0=None
pose={'x':0.0,'y':0.0,'h':0.0}
def upd():
    global o0
    o1=R.odom()
    if o0 is None: o0=o1; return pose['x'],pose['y'],pose['h']
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
    pose['h']=R.heading()
    pose['x']+=d*math.cos(math.radians(pose['h']))
    pose['y']+=d*math.sin(math.radians(pose['h']))
    o0=o1
    return pose['x'],pose['y'],pose['h']

GRID={}
def mark(x,y):
    k=(int(x*4),int(y*4))
    GRID[k]=GRID.get(k,0)+1

def coverage():
    return len(GRID)

FRONT=0.36
RIGHT_T=0.26
def follow_step(l):
    front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
    r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9.0])
    lm=min([x for x in (l[1],l[2],l[3]) if x>0] or [9.0])
    if front<0.15:
        R.stop(); return 'stop'
    if front<FRONT:
        # rotate toward open side; default left (right-hand rule turns left at dead ends? no:
        # right-hand rule = keep wall on RIGHT; at dead end turn LEFT (CCW) if open
        if lm>r+0.05: R.motors(-22,22)
        elif r>lm+0.05: R.motors(22,-22)
        else: R.motors(-22,22)
        return 'turn'
    if r>1.0 and l[14]>1.0 and l[15]>1.0:
        R.motors(20,-10)  # curve right to hug wall
        return 'seekR'
    corr=(r-RIGHT_T)*50
    corr=max(-13,min(13,corr))
    base=21
    R.motors(base+corr, base-corr)
    return 'go'

def main():
    L('EXPLORE5 start')
    last_rep=0; stuck=0; homing=False
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        d0=R.fget('d0')
        if d0!=0:
            L('*** D0=%s *** lidar=%s'%(d0,R.get('d2'))); R.stop(); time.sleep(1.5); continue
        txs=txstate()
        # quick check for rx messages handled by reader cache; log any
        m=R.rx()
        if m: L('RX!! %s'%m.replace('\n',' | '))
        d11now=R.fget('d11')
        if not homing and txs=='busy' and 0.55<d11now<0.75:
            if time.time()-MYTX['t']>0.7:
                homing=True; L('-> HOMING (busy, d11=%.3f)'%d11now)
        elif homing and (txs!='busy' or d11now>=0.8 or d11now<=0.55) and time.time()-MYTX['t']>0.7:
            homing=False; L('-> EXPLORE (d11=%.3f)'%d11now)
        if homing:
            # gentle homing: probe 3 headings, go best
            s0=R.fget('d11'); h=R.heading()
            best=(s0,0)
            for dgt in (0,50,-50):
                tgt=(h+dgt)%360
                t0=time.time()
                while time.time()-t0<2.5:
                    err=((tgt-R.heading()+180)%360)-180
                    if abs(err)<5: break
                    s=24 if err>0 else -24
                    R.motors(s,-s); time.sleep(0.07)
                R.stop(); time.sleep(0.15)
                s1=R.fget('d11')
                if s1>best[0]: best=(s1,dgt)
                # undo turn
                t0=time.time()
                while time.time()-t0<2.5:
                    err=((h-R.heading()+180)%360)-180
                    if abs(err)<5: break
                    s=24 if err>0 else -24
                    R.motors(s,-s); time.sleep(0.07)
                R.stop()
            tgt=(R.heading()+best[1])%360
            t0=time.time()
            while time.time()-t0<2:
                err=((tgt-R.heading()+180)%360)-180
                if abs(err)<5: break
                s=24 if err>0 else -24
                R.motors(s,-s); time.sleep(0.07)
            R.motors(22,22); time.sleep(1.2); R.stop()
            L('HOMING best d=%+d sig %.3f->%.3f'%(best[1],s0,best[0]))
            continue
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        mode=follow_step(l)
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        if mode=='stop':
            stuck+=1
            if stuck>6:
                L('STUCK RECOVERY')
                R.motors(-24,-24); time.sleep(0.8)
                lm=min([x for x in (l[1],l[2],l[3]) if x>0] or [9])
                r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9])
                if lm>r: R.motors(-24,24)
                else: R.motors(24,-24)
                time.sleep(0.8); R.stop(); stuck=0
        else:
            stuck=max(0,stuck-1)
        x,y,h=upd()
        mark(x,y)
        if time.time()-MYTX['t']>2.5: beacon_tick()
        if time.time()-last_rep>30:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s front=%.2f cov=%d tx=%s d11=%.3f lidar=%s'%(
                x,y,h,mode,front,coverage(),txstate(),R.fget('d11'),','.join('%.1f'%v for v in l)))
        time.sleep(0.07)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE5 end cov=%d'%coverage())
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid.json','w'))
