import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore8.log','a')
MAP=open('/memory/map8.log','a')
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

MYTX={'t':0.0}
def beacon_tick():
    msgs=[
      'B-TO-A: B ALIVE, SEARCHING GOAL. REPLY IF YOU HEAR.',
      'B-TO-A: IF GOAL FOUND SAY GOAL AT HEADING <deg>.',
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
pose={'x':0.0,'y':0.0,'h':0.0}
def upd():
    global o0
    o1=R.odom()
    if o0 is None:
        o0=o1; return
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
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

def map_log(l):
    # log world-frame ray endpoints
    h=math.radians(pose['h'])
    pts=[]
    for k in range(16):
        d=l[k]
        if d<=0: continue
        ang=h-math.radians(22.5*k)  # beam k at +22.5k CCW => world angle h+22.5k? CCW from front with CW-positive compass => world = h - 22.5k
        pts.append('%.2f,%.2f:%.2f'%(pose['x']+d*math.cos(ang),pose['y']+d*math.sin(ang),d))
    MAP.write('P %.2f %.2f %.1f | %s\n'%(pose['x'],pose['y'],pose['h'],' '.join(pts))); MAP.flush()

def main():
    L('EXPLORE8 start')
    last_rep=0; last_bc=0
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
        upd()
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        if front>=0.45:
            r=min([x for x in (l[13],l[14],l[15]) if x>0] or [9.0])
            corr=(r-0.24)*40
            corr=max(-11,min(11,corr))
            R.motors(19+corr, 19-corr)
            mode='go'
            time.sleep(0.35)
        elif front>=0.25:
            R.motors(17,17); mode='creep'; time.sleep(0.22)
        else:
            left=avgc(l,range(1,6)); right=avgc(l,range(11,16))
            if front<0.17:
                R.motors(-20,-20); time.sleep(1.3); R.stop(); mode='back'
                l=med_lidar() or l
            if left>=right:
                R.motors(-11,-25); time.sleep(2.2); R.stop(); mode='archL(%.2f/%.2f)'%(left,right)
            else:
                R.motors(-25,-11); time.sleep(2.2); R.stop(); mode='archR(%.2f/%.2f)'%(left,right)
            upd(); mark()
        upd(); mark()
        map_log(l)
        d11=R.fget('d11')
        if time.time()-MYTX['t']>7.0: beacon_tick()
        if time.time()-last_rep>40:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d tx=%s d11=%.3f lidar=%s'%(
                pose['x'],pose['y'],pose['h'],mode,len(GRID),txstate(),d11,','.join('%.1f'%v for v in l)))
        time.sleep(0.04)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE8 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid8.json','w'))
