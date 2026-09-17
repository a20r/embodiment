import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore6.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=[
      'B-TO-A: B HERE. STATUS: MOBILE, SEARCHING GOAL. REPLY ANYTHING.',
      'B-TO-A: IF YOU HEAR THIS, SEND YOUR d11 AND STATUS. I LISTEN.',
      'B-TO-A: A? BEEP BACK IF ALIVE.',
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
    k=(int(x*4),int(y*4)); GRID[k]=GRID.get(k,0)+1

FRONT=0.36; RIGHT_T=0.26
def follow_step(l):
    front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
    r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9.0])
    lm=min([x for x in (l[1],l[2],l[3]) if x>0] or [9.0])
    if front<0.15: R.stop(); return 'stop'
    if front<FRONT:
        if lm>r+0.05: R.motors(-22,22)
        elif r>lm+0.05: R.motors(22,-22)
        else: R.motors(-22,22)
        return 'turn'
    if r>1.0 and l[14]>1.0:
        R.motors(20,-10); return 'seekR'
    corr=(r-RIGHT_T)*50
    corr=max(-13,min(13,corr))
    base=21
    R.motors(base+corr, base-corr)
    return 'go'

def escape(l):
    # wedged: reverse out toward the most open rear direction, then rotate
    rear=[x for x in (l[7],l[8],l[9],l[10]) if x>0]
    L('ESCAPE rear=%s l=%s'%([round(x,2) for x in rear],[round(x,2) for x in l]))
    R.motors(-26,-26); time.sleep(1.1); R.stop()
    l2=med_lidar() or l
    # choose rotation: the side (left beams 1-5 vs right beams 11-15) with more clearance
    lm=sum(x for x in l2[1:6] if x>0)/max(1,len([x for x in l2[1:6] if x>0]))
    rm=sum(x for x in l2[11:16] if x>0)/max(1,len([x for x in l2[11:16] if x>0]))
    if lm>=rm: R.motors(-24,24)
    else: R.motors(24,-24)
    time.sleep(1.2); R.stop()
    time.sleep(0.2)

def main():
    L('EXPLORE6 start h=%.1f d11=%.3f'%(R.heading(),R.fget('d11')))
    last_rep=0; stuck=0; last_d11=0.669; turn_since=0
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
        mode=follow_step(l)
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        if mode in ('stop','turn'):
            turn_since+=1
        else:
            turn_since=0
        if mode=='stop':
            stuck+=1
        else:
            stuck=max(0,stuck-1)
        if turn_since>28:
            L('WEDGED (turn_since=%d)'%turn_since)
            escape(l); turn_since=0; stuck=0
        x,y,h=upd(); mark(x,y)
        d11=R.fget('d11')
        if abs(d11-last_d11)>0.02:
            L('d11 CHANGED %.3f -> %.3f'%(last_d11,d11)); last_d11=d11
        if time.time()-MYTX['t']>5.0: beacon_tick()
        if time.time()-last_rep>45:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s front=%.2f cov=%d tx=%s d11=%.3f lidar=%s'%(
                x,y,h,mode,front,len(GRID),txstate(),d11,','.join('%.1f'%v for v in l)))
        time.sleep(0.07)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE6 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid6.json','w'))
