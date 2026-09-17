import sys,time,math,statistics,json,random
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/ep2.log','a')
def L(s):
    try:
        LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass

random.seed()
pose={'x':0.0,'y':0.0,'h':None,'dist':0.0}
o0=None
def upd():
    global o0
    o1=R.odom()
    if o0 is None: o0=o1; return
    dl=o1[0]-o0[0]; dr=o1[1]-o0[1]
    if abs(dl)>500 or abs(dr)>500: o0=o1; return
    o0=o1
    d=(dl+dr)/2*0.00066
    pose['dist']+=abs(d)
    h=R.heading()
    if pose['h'] is None: pose['h']=h
    else:
        dh=((h-pose['h']+180)%360)-180
        if abs(dh)>90: return
        pose['h']=h
    pose['x']+=d*math.cos(math.radians(pose['h']))
    pose['y']+=d*math.sin(math.radians(pose['h']))

GRID=set()
def mark():
    GRID.add((int(pose['x']*2),int(pose['y']*2)))

def med_lidar(n=3,tries=30):
    scans=[]
    for _ in range(tries):
        s=R.lidar()
        if len(s)==16: scans.append(s)
        if len(scans)>=n: break
        time.sleep(0.03)
    if not scans: return None
    return [statistics.median([s[i] for s in scans]) for i in range(16)]

def txstate():
    st=R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':')[1] if ':' in tok else '?'
    return '?'

def sig(n=6,dt=0.06):
    vs=[]
    for _ in range(n):
        v=R.fget('d11')
        if v>0: vs.append(v)
        time.sleep(dt)
    return statistics.median(vs) if vs else 0

STATE={'beacon_n':0,'peer':False,'n':0}
def beacon(msg):
    STATE['beacon_n']+=1
    try: R.tx('B-TO-A: %s #%d'%(msg,STATE['beacon_n']))
    except Exception: pass

def turn_to(tgt,speed=24,timeout=4):
    t0=time.time()
    while time.time()-t0<timeout:
        err=((tgt-R.heading()+180)%360)-180
        if abs(err)<6: break
        if err>0: R.motors(speed,-speed)
        else: R.motors(-speed,speed)
        time.sleep(0.06)
    R.stop(); time.sleep(0.1)

def drive_hdg(tgt,speed=20,dur=6.0):
    t0=time.time()
    while True:
        err=((tgt-R.heading()+180)%360)-180
        if err>8: R.motors(speed,-int(speed*0.8))
        elif err<-8: R.motors(-int(speed*0.8),speed)
        else: R.motors(speed,speed)
        l=R.lidar()
        f=9.0
        if len(l)==16:
            f=min([x for x in (l[15],l[0],l[1]) if x>0] or [9])
        if f<0.30: R.stop(); return l
        if dur and time.time()-t0>dur: R.stop(); return l
        time.sleep(0.08)

def peer_near_action():
    L('*** PEER IN RADIO RANGE (tx=%s) - stopping, beacon burst ***'%txstate())
    R.stop()
    for i in range(6):
        beacon('RANGE! STOPPED. WHERE ARE YOU? SEND POS/HDG')
        m=R.rx()
        if m: L('RX!! %s'%m[:120])
        time.sleep(0.7)
    st=txstate()
    L('after burst tx=%s'%st)
    if st in ('ok','busy'): STATE['peer']=True

def main():
    L('EP2 start d11=%.3f tx=%s'%(sig(),txstate()))
    last_be=0; last_sum=0; last_grid=0; stuck=0; last_turn=0
    tgt=R.heading()
    while True:
        STATE['n']+=1
        try:
            st=R.status_d()
            if st.get('here')=='1':
                L('*** HERE=1 AT GOAL? %s'%json.dumps(st))
                R.stop()
                for i in range(8):
                    beacon('AT-GOAL? here=1. A COME')
                    time.sleep(0.6)
                time.sleep(2); continue
            g=st.get('goal','0')
            if g not in ('0',''):
                L('*** GOAL FLAG %s'%json.dumps(st))
            m=R.rx()
            if m: L('RX!! %s'%m[:150])
            ts=txstate()
            if ts in ('ok','busy') and not STATE['peer']:
                peer_near_action()
            if time.time()-last_be>2.2:
                last_be=time.time()
                beacon('B alive pos=%.1f,%.1f hdg=%.0f seek A+GOAL'%(pose['x'],pose['y'],pose['h'] or R.heading()))
            l=med_lidar()
            if l is None: time.sleep(0.05); continue
            upd(); mark()
            s=sig(3,0.05)
            if s>0.55: L('d11 HIGH %.3f pos=%.1f,%.1f'%(s,pose['x'],pose['y']))
            # steering
            if len(l)==16:
                def cl(i): 
                    v=l[i%16]; return v if v>0 else 2.0
                front=min(cl(15),cl(0),cl(1))
                left=min(cl(3),cl(4),cl(5)); right=min(cl(11),cl(12),cl(13))
                if front<0.32:
                    R.stop()
                    if left>right+0.1: tgt=(R.heading()+85)%360
                    elif right>left+0.1: tgt=(R.heading()-85)%360
                    else: tgt=(R.heading()+ (90 if random.random()<0.5 else -90))%360
                    L('TURN front=%.2f L=%.2f R=%.2f -> tgt=%.0f'%(front,left,right,tgt))
                    turn_to(tgt)
                    stuck+=1
                elif left<0.16 and right>=left: tgt=(R.heading()+12)%360
                elif right<0.16 and left>right: tgt=(R.heading()-12)%360
                l2=drive_hdg(tgt,speed=20)
                L("SEG tgt=%.0f front=%s"%(tgt,[round(x,2) for x in (l2[15],l2[0],l2[1])] if l2 and len(l2)==16 else "?"))
            if time.time()-last_sum>30:
                last_sum=time.time()
                L('SUM pos=%.1f,%.1f h=%.0f dist=%.1f cells=%d d11=%.3f tx=%s'%(pose['x'],pose['y'],pose['h'] or -1,pose['dist'],len(GRID),s,txstate()))
            if len(GRID)==last_grid and time.time()-last_turn>40:
                last_turn=time.time()
                tgt=(R.heading()+random.choice([120,150,-120,-150]))%360
                turn_to(tgt)
                L('WANDER turn -> %.0f'%tgt)
            elif len(GRID)!=last_grid:
                last_grid=len(GRID); last_turn=time.time()
        except Exception as e:
            try:
                R.stop(); L('EXC %r'%(e,))
            except Exception: pass
            time.sleep(0.5)

if __name__=='__main__':
    main()
