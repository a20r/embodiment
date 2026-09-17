import sys,time,statistics,math,json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore10.log','a')
MAP=open('/memory/map10.log','a')
SIG=open('/memory/sig10.log','a')
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

def med_lidar(n=2,tries=25):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.04)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

def sig_med(n=12,dt=0.08):
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

def steer_toward(H, base=19):
    err=((H-pose['h']+180)%360)-180
    corr=max(-10,min(10, -err*0.8))
    R.motors(base-corr, base+corr)

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
    L('EXPLORE10 start')
    H=R.heading(); odo_mark=0.0; legs=0
    last_rep=0
    while True:
        st=R.status_d()
        if st.get('here')=='1':
            L('*** HERE=1 *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL *** %s'%json.dumps(st)); R.stop(); time.sleep(2); continue
        d0=R.fget('d0')
        if d0!=0:
            L('*** D0=%s ***'%d0); R.stop(); time.sleep(1.5); continue
        m=R.rx()
        if m: L('RX!! %s'%m.replace('\n',' | ')[:120])
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.05); continue
        upd(); mark(); map_log(l)
        s=sig_med()
        SIG.write('%.2f %.2f %.3f %s %s\n'%(pose['x'],pose['y'],s,txstate(),time.time()-MYTX['t']>1.5 and 'quiet' or 'selftx')); SIG.flush()
        # hotspot check (only when channel quiet for me recently)
        if s>0.62 and time.time()-MYTX['t']>2.0:
            L('HOTSPOT d11=%.3f at %.2f,%.2f'%(s,pose['x'],pose['y']))
            R.stop()
            # investigate: burst + listen
            for i in range(6):
                try: R.tx('B-TO-A: I SEE A SIGNAL. WHERE ARE YOU?')
                except Exception: pass
                time.sleep(0.25)
            s2=sig_med(20)
            L('HOTSPOT recheck %.3f'%s2)
            if s2>0.62:
                # try to home: probe 6 directions
                best=(s2,0)
                h0=R.heading()
                for dg in (0,60,-60,120,-120,180):
                    tgt=(h0+dg)%360
                    t0=time.time()
                    while time.time()-t0<2.5:
                        err=((tgt-R.heading()+180)%360)-180
                        if abs(err)<5: break
                        R.motors(24,-24) if err>0 else R.motors(-24,24)
                        time.sleep(0.07)
                    R.stop()
                    sv=sig_med(10)
                    if sv>best[0]: best=(sv,dg)
                    t0=time.time()
                    while time.time()-t0<2.5:
                        err=((h0-R.heading()+180)%360)-180
                        if abs(err)<5: break
                        R.motors(24,-24) if err>0 else R.motors(-24,24)
                        time.sleep(0.07)
                    R.stop()
                L('HOTSPOT best d=%+d s=%.3f'%(best[1],best[0]))
            continue
        front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
        if front>=0.33:
            steer_toward(H); mode='leg'
        else:
            if front<0.3:
                # back straight out first until room
                R.motors(-20,-20); time.sleep(1.2); R.stop(); upd()
                l2=med_lidar() or l
                front2=min([x for x in (l2[15],l2[0],l2[1]) if x>0] or [0])
                if front2<0.4:
                    R.motors(-20,-20); time.sleep(1.2); R.stop(); upd()
                    l2=med_lidar() or l2
                l=l2
                front=min([x for x in (l[15],l[0],l[1]) if x>0] or [0])
            left=avgc(l,range(1,7)); right=avgc(l,range(10,16))
            if left>=right:
                R.motors(-12,-26); time.sleep(2.0); R.stop(); mode='archL'
            else:
                R.motors(-26,-12); time.sleep(2.0); R.stop(); mode='archR'
            upd()
        if time.time()-MYTX['t']>7.0: beacon_tick()
        if pose['dist']-odo_mark>1.4:
            odo_mark=pose['dist']; legs+=1
            H=(H+90)%360
            L('LEG %d heading %d dist=%.2f cov=%d'%(legs,H,pose['dist'],len(GRID)))
        if time.time()-last_rep>40:
            last_rep=time.time()
            L('POS %.2f,%.2f h=%.0f %s cov=%d dist=%.2f tx=%s d11=%.3f lidar=%s'%(
                pose['x'],pose['y'],pose['h'],mode,len(GRID),pose['dist'],txstate(),s,','.join('%.1f'%v for v in l)))
        time.sleep(0.04)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE10 end cov=%d'%len(GRID))
        json.dump({'%d,%d'%k:v for k,v in GRID.items()}, open('/memory/grid10.json','w'))
