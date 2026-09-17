import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore9.log','a')
MAP=open('/memory/map9.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=[
      'B-TO-A: B ALIVE. SEARCHING GOAL. REPLY IF YOU HEAR.',
      'B-TO-A: IF GOAL FOUND SAY: GOAL AT HEADING <deg>.',
      'B-TO-A: BEEP.',
    ]
    try:
        R.tx(msgs[int(time.time())%3]); MYTX['t']=time.time()
    except Exception: pass

def med_lidar(n=3,tries=30):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.05)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

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

def steer_toward(H, base=18):
    err=((H-pose['h']+180)%360)-180
    if abs(err)<4: corr=0
    else: corr=max(-10,min(10, -err*0.8))
    # CW (+err) needs L>R: corr negative when err>0? L=base+corr...
    R.motors(base-corr, base+corr)

def main():
    L('EXPLORE9 lawnmower start')
    H=R.heading()
    leg=0.0; odo_mark=0.0
    last_rep=0; last_bc=0; legs=0
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        d0=R.fget('d0')
        if d0!=0:
            L('*** D0=%s *** lidar=%s'%(d0,R.get('d2'))); R.stop(); time.sleep(1.5); continue
        m=R.rx()
        if m: L('RX!! %s'%m.replace('\n',' | ')[:120])
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd(); mark()
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        if front>=0.33:
            steer_toward(H)
            mode='leg%.1f'%leg
        else:
            # blocked: arch around obstacle keeping general direction
            left=avgc(l,range(1,7)); right=avgc(l,range(10,16))
            if front<0.18:
                R.motors(-20,-20); time.sleep(1.2); R.stop()
                upd()
            if left>=right:
                R.motors(-11,-25); time.sleep(2.0); R.stop(); mode='archL'
            else:
                R.motors(-25,-11); time.sleep(2.0); R.stop(); mode='archR'
            upd()
        if time.time()-MYTX['t']>7.0: beacon_tick()
        # leg length: after ~1.6m of travel, rotate leg heading by 90
        if pose['dist']-odo_mark>1.6:
            odo_mark=pose['dist']; legs+=1
            H=(H+90)%360
            L('LEG %d new heading %d dist=%.2f cov=%d'%(legs,H,pose['dist'],len(GRID)))
        if time.time()-last_rep>40:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d dist=%.2f tx=%s d11=%.3f lidar=%s'%(
                pose['x'],pose['y'],pose['h'],mode,len(GRID),pose['dist'],txstate(),R.fget('d11'),','.join('%.1f'%v for v in l)))
        time.sleep(0.04)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE9 end cov=%d dist=%.2f'%(len(GRID),pose['dist']))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid9.json','w'))
