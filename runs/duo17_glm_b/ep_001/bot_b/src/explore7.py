import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore7.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=[
      'B-TO-A: B HERE. MOBILE, SEARCHING GOAL. REPLY IF ALIVE.',
      'B-TO-A: SEND YOUR d11 AND STATUS WHEN YOU HEAR THIS.',
      'B-TO-A: A? BEEP.',
    ]
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
    if o0 is None:
        o0=o1; return pose['x'],pose['y'],pose['h']
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
    pose['h']=R.heading()
    pose['x']+=d*math.cos(math.radians(pose['h']))
    pose['y']+=d*math.sin(math.radians(pose['h']))
    o0=o1
    return pose['x'],pose['y'],pose['h']

GRID={}
def mark(x,y):
    k=(int(x*3),int(y*3)); GRID[k]=GRID.get(k,0)+1

def avg(l,idxs):
    v=[l[i] for i in idxs if 0<l[i]<1.5]
    return sum(v)/len(v) if v else 1.5

# maneuvers
def fwd(t=0.3,spd=22):
    R.motors(spd,spd); time.sleep(t); R.stop()
def back(t=0.8,spd=24):
    R.motors(-spd,-spd); time.sleep(t); R.stop()
def back_left(t=1.1):
    R.motors(-12,-26); time.sleep(t); R.stop()
def back_right(t=1.1):
    R.motors(-26,-12); time.sleep(t); R.stop()
def turn_ccw(t=0.8):
    R.motors(-24,24); time.sleep(t); R.stop()
def turn_cw(t=0.8):
    R.motors(24,-24); time.sleep(t); R.stop()

def step(l):
    front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
    r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9.0])
    if front>=0.32:
        corr=(r-0.22)*45
        corr=max(-12,min(12,corr))
        base=21
        R.motors(base+corr, base-corr)
        return 'go'
    # blocked ahead: find best opening among beams 2..14 within reach
    best_k=None; best_v=-1
    for k in range(2,15):
        v=l[k]
        if 0<v and v>best_v:
            # require some continuity
            nb=[l[(k+j)%16] for j in (-1,0,1)]
            if min([x for x in nb if x>0] or [0])>0.35:
                best_v=v; best_k=k
    if best_k is None:
        back(0.9); return 'backup'
    ang=22.5*best_k if best_k<=8 else 22.5*(best_k-16)  # signed CCW angle
    if front<0.18: back(0.7)
    if ang>15:
        back_left(0.9+min(1.2,abs(ang)/40))
        return 'archL%d'%best_k
    elif ang<-15:
        back_right(0.9+min(1.2,abs(ang)/40))
        return 'archR%d'%best_k
    else:
        # opening near-front but blocked front: shuffle forward slow
        fwd(0.35,16)
        return 'shuffle'

def main():
    L('EXPLORE7 start h=%.1f'%(R.heading()))
    last_rep=0; last_cov_grow=0; cov_prev=0; last_d11=0.52
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
        mode=step(l)
        x,y,h=upd(); mark(x,y)
        d11=R.fget('d11')
        if abs(d11-last_d11)>0.05:
            L('d11 %.3f -> %.3f'%(last_d11,d11)); last_d11=d11
        if len(GRID)>cov_prev:
            cov_prev=len(GRID); last_cov_grow=time.time()
        if time.time()-last_cov_grow>25:
            L('NO COVERAGE GROWTH 25s - big escape')
            back(1.3); turn_cw(1.0); last_cov_grow=time.time()
        if time.time()-MYTX['t']>6.0: beacon_tick()
        if time.time()-last_rep>45:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d tx=%s d11=%.3f lidar=%s'%(
                x,y,h,mode,len(GRID),txstate(),d11,','.join('%.1f'%v for v in l)))
        time.sleep(0.05)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE7 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid7.json','w'))
