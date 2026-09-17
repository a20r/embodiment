import sys, time, statistics, math, json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore4.log','a')
MAP=open('/memory/map.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=['B-TO-A: EXPLORING, COME TO ME OR KEEP TXING.',
          'B-TO-A: WHERE ARE YOU? REPLY ANYTHING.',
          'B-TO-A: GOAL FOUND? REPLY GOAL YES/NO.']
    try:
        R.tx(msgs[int(time.time())%3]); MYTX['t']=time.time()
    except Exception: pass

def med_lidar(n=2, tries=25):
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

def sig(n=5):
    vs=[R.fget('d11') for _ in range(n)]
    vs=[v for v in vs if v>0]
    return statistics.median(vs) if vs else 0.0

o0=R.odom()
pose={'x':0.0,'y':0.0,'h':R.heading()}
def upd():
    o1=R.odom()
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
    pose['h']=R.heading()
    pose['x']+=d*math.cos(math.radians(pose['h']))
    pose['y']+=d*math.sin(math.radians(pose['h']))
    return pose['x'],pose['y'],pose['h']

def peer_busy():
    t0=time.time(); seen=0; n=0
    while time.time()-t0<1.3:
        if time.time()-MYTX['t']<0.6:
            time.sleep(0.1); continue
        n+=1
        if txstate()=='busy': seen+=1
        time.sleep(0.08)
    return n>0 and seen/max(1,n)>=0.6

def homing_step():
    s0=sig(); h=R.heading()
    best=(s0,h); bestoff=0
    for d in (0,40,-40,80,-80,140,-140):
        tgt=(h+d)%360
        t0=time.time()
        while time.time()-t0<2.5:
            err=((tgt-R.heading()+180)%360)-180
            if abs(err)<5: break
            s=26 if err>0 else -26
            R.motors(s,-s); time.sleep(0.07)
        R.stop()
        s1=sig()
        if s1>best[0]: best=(s1,tgt); bestoff=d
    L('HOMING best %.3f off %+d h=%.0f'%(best[0],bestoff,h))
    t0=time.time()
    while time.time()-t0<2:
        err=((best[1]-R.heading()+180)%360)-180
        if abs(err)<6: break
        s=26 if err>0 else -26
        R.motors(s,-s); time.sleep(0.07)
    R.motors(24,24); time.sleep(1.3); R.stop()
    L('HOMING after drive sig=%.3f'%sig())

RIGHT=0.24   # target right-wall distance
FRONT=0.34   # min front clearance to proceed
def follow_step(l):
    f15,f0,f1=l[15],l[0],l[1]
    front=min([x for x in (f15,f0,f1) if x>0] or [0])
    r_side=[x for x in (l[13],l[14],l[15]) if x>0]
    r=min(r_side) if r_side else 9.0
    l_side=[x for x in (l[1],l[2],l[3]) if x>0]
    lm=min(l_side) if l_side else 9.0
    if front<FRONT:
        # too close ahead: turn toward more open side (CCW preferred = left-hand at dead end -> right-hand rule says turn left? use side compare)
        if lm>=r:
            R.motors(-26,26)   # CCW left
        else:
            R.motors(26,-26)   # CW right
        return 'turn'
    # steer to keep right wall
    err=r-RIGHT  # >0 too far from wall -> steer right (CW, negative correction)
    if r>1.1:
        # right wall lost: turn right to find it
        R.motors(20,-14); return 'seek-right'
    corr=max(-14,min(14, (RIGHT-r)*60 if r<=1.1 else 0))
    base=22
    R.motors(base-corr, base+corr)  # corr>0 => left slower? test: CW turn = left fwd right back => motors(L,R)=(+,-): so corr>0 (need steer right/CW) => L=base-corr, R=base+corr?? no
    return 'go'

def follow_step2(l):
    f15,f0,f1=l[15],l[0],l[1]
    front=min([x for x in (f15,f0,f1) if x>0] or [0])
    r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9.0])
    lm=min([x for x in (l[1],l[2],l[3]) if x>0] or [9.0])
    if front<FRONT:
        if lm>=r: R.motors(-26,26)
        else: R.motors(26,-26)
        return 'turn'
    if r>1.1:
        R.motors(20,-12)  # curve right to reacquire wall
        return 'seek'
    corr=(RIGHT-r)*55
    corr=max(-15,min(15,corr))
    # CW (right turn) = left fwd faster, right slower => motors(+c,-c) increases heading
    # if too far from right wall (r>RIGHT): corr>0 => steer right => L=base+corr, R=base-corr
    base=22
    R.motors(base+corr, base-corr)
    return 'go'

def main():
    L('EXPLORE4 start h=%.1f'%R.heading())
    last_rep=0; last_map=0; stuck_t=0; homing=False
    last_lidar=None
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(1); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL FLAG *** %s'%json.dumps(st)); R.stop(); time.sleep(1); continue
        d0=R.fget('d0'); d5=R.fget('d5')
        if d0!=0 or d5!=0:
            L('SENSOR! d0=%s d5=%s lidar=%s'%(d0,d5,R.get('d2')))
        txs=txstate()
        if not homing and txs=='busy' and peer_busy():
            homing=True; L('-> HOMING')
        elif homing and not peer_busy():
            homing=False; L('-> EXPLORE')
        if homing:
            homing_step()
        else:
            l=med_lidar()
            if l is None: R.stop(); time.sleep(0.05); continue
            mode=follow_step2(l)
            front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
            if front<0.14:
                stuck_t+=1
            else:
                stuck_t=0
            if stuck_t>12:
                L('STUCK: backup+spin')
                R.motors(-26,-26); time.sleep(0.9)
                R.motors(-24,24); time.sleep(0.9)
                R.stop(); stuck_t=0
            if time.time()-MYTX['t']>2.5: beacon_tick()
            if time.time()-last_rep>25:
                last_rep=time.time()
                x,y,h=upd()
                L('POS %.2f,%.2f h=%.0f %s front=%.2f sig=%.3f tx=%s d0=%s d5=%s lidar=%s'%(
                    x,y,h,mode,front,sig(),txstate(),R.get('d0'),R.get('d5'),','.join('%.1f'%v for v in l)))
            if time.time()-last_map>1.0:
                last_map=time.time()
                x,y,h=upd()
                MAP.write('%.2f %.2f %.1f %s\n'%(x,y,h,','.join('%.2f'%v for v in l))); MAP.flush()
        time.sleep(0.07)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE4 end')
