import sys, time, statistics, math, json
sys.path.insert(0,'/bot/src')
import robot2 as R

LOG=open('/memory/explore3.log','a')
MYTX={'t':0.0}
def beacon_tick():
    msgs=[
     'B-TO-A: I AM EXPLORING TOWARD YOU. KEEP TRANSMITTING.',
     'B-TO-A: SEND ANY MESSAGE, I LISTEN BETWEEN MY PINGS.',
     'B-TO-A: REPLY WITH YOUR STATUS IF YOU CAN.',
    ]
    try:
        R.tx(msgs[int(time.time())%3]); MYTX['t']=time.time()
    except Exception: pass
def L(s):
    LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()

def med_lidar(n=2, tries=30):
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

def peer_busy():
    t0=time.time(); seen=0; n=0
    while time.time()-t0<1.4:
        if time.time()-MYTX['t']<0.6:
            time.sleep(0.1); continue
        n+=1
        if txstate()=='busy': seen+=1
        time.sleep(0.08)
    return n>0 and seen/max(1,n)>=0.6

def sig(n=6):
    vs=[R.fget('d11') for _ in range(n)]
    vs=[v for v in vs if v>0]
    return statistics.median(vs) if vs else 0.0

# pose integration
o0=R.odom(); pose={'x':0.0,'y':0.0}
def upd():
    o1=R.odom()
    d=((o1[0]-o0[0])+(o1[1]-o0[1]))/2*0.00066
    h=R.heading()
    pose['x']+=d*math.cos(math.radians(h))
    pose['y']+=d*math.sin(math.radians(h))
    return pose['x'],pose['y'],h

def steererr(l):
    # keep centered: right wall = beams 13,14,15 (22.5*13..15 CCW = left?? no: CCW from front means
    # beam k at +22.5k => beams 14,15 are RIGHT side (i.e., -45,-22.5 => +315,+337.5)
    right=min([x for x in (l[14],l[15]) if x>0] or [9])
    left=min([x for x in (l[1],l[2]) if x>0] or [9])
    return left-right  # >0 => more room on right => steer right (heading decrease? CW is +)

def homing_step():
    # hill-climb while channel busy
    s0=sig()
    h=R.heading()
    best=(s0,h)
    for d in (0,45,-45,90,-90,180):
        tgt=(h+d)%360
        t0=time.time()
        while time.time()-t0<3:
            err=((tgt-R.heading()+180)%360)-180
            if abs(err)<5: break
            s=28 if err>0 else -28
            R.motors(s,-s); time.sleep(0.07)
        R.stop()
        s1=sig()
        if s1>best[0]: best=(s1,tgt)
    L('HOMING best %.3f at h=%.0f (s0 %.3f)'%(best[0],best[1],s0))
    # drive toward best heading
    t0=time.time()
    while time.time()-t0<2.5:
        err=((best[1]-R.heading()+180)%360)-180
        if abs(err)<6: break
        s=28 if err>0 else -28
        R.motors(s,-s); time.sleep(0.07)
    drive_secs=1.5
    R.motors(26,26); time.sleep(drive_secs); R.stop()
    s2=sig()
    L('HOMING after drive sig=%.3f'%s2)

def main():
    L('EXPLORE3 start')
    last_report=0
    homing_mode=False
    while True:
        st=R.status_d()
        txs=txstate()
        if st.get('here')=='1':
            L('*** HERE=1 *** '+json.dumps(st)); time.sleep(0.5); continue
        if st.get('goal','0') not in ('0',''):
            L('*** GOAL FLAG *** '+json.dumps(st)); time.sleep(0.5); continue
        d0=R.fget('d0'); d5=R.fget('d5')
        if d0!=0 or d5!=0:
            L('*** D0/D5 nonzero: %s %s ***'%(d0,d5))
        if not homing_mode and txs=='busy':
            if peer_busy():
                homing_mode=True; L('PEER CARRIER -> homing mode')
        elif homing_mode:
            if txstate()!='busy' and time.time()-MYTX['t']>0.6:
                homing_mode=False; L('quiet -> explore mode')
        if homing_mode:
            homing_step()
            continue
        l=med_lidar()
        if l is None: R.stop(); time.sleep(0.1); continue
        f=[x for x in (l[15],l[0],l[1]) if x>0]
        front=min(f) if f else 0
        if front>0.42:
            e=steererr(l)
            gain=max(-12,min(12,-e*14))
            R.motors(24-gain,24+gain)  # gain>0 => steer CCW(left fwd faster? careful)
            upd()
        else:
            # pick side with more clearance and turn
            right=min([x for x in (l[12],l[13],l[14],l[15]) if x>0] or [0])
            left=min([x for x in (l[1],l[2],l[3],l[4]) if x>0] or [0])
            if right>=left:
                R.motors(30,-30)  # CW
            else:
                R.motors(-30,30)  # CCW
            upd()
        if time.time()-MYTX['t']>2.0:
            beacon_tick()
        if time.time()-last_report>20:
            last_report=time.time()
            x,y,h=upd()
            L('POS %.2f,%.2f h=%.0f front=%.2f sig=%.3f tx=%s batt=%s d0=%s d5=%s lidar=%s'%(
                x,y,h,front,sig(),txstate(),R.get('d11'),R.get('d0'),R.get('d5'),
                ','.join('%.1f'%v for v in l)))
        time.sleep(0.06)

if __name__=='__main__':
    try: main()
    finally:
        R.stop(); L('EXPLORE3 end pos=%s'%json.dumps(pose))
