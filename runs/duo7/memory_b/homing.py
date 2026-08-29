import time, math, sys, threading, random
sys.path.insert(0,'/bot/src')
from lib import rd, wr, drive, stop, tx

def log(m):
    with open('/tmp/hom.log','a') as f: f.write('%.1f %s\n'%(time.time(),m))

sig={'v':0.0,'n':0}
def d5_thread():
    buf=[]; last=time.time()
    with open('/dev/robot/d5') as f:
        while True:
            line=f.readline().strip()
            if line:
                try: buf.append(float(line))
                except: pass
            now=time.time()
            if now-last>1.0 and buf:
                sig['v']=sum(buf)/len(buf); sig['n']+=1
                buf=[]; last=now
threading.Thread(target=d5_thread,daemon=True).start()

def lidar(prev=[None]):
    while True:
        try:
            v=[float(x) for x in rd('d3').split(',')]
            if len(v)==16:
                if prev[0]:
                    v=[v[i] if v[i]>=0 else prev[0][i] for i in range(16)]
                else:
                    v=[x if x>=0 else 3.0 for x in v]
                prev[0]=v
                return v
        except: pass
def goal():
    try: return int(rd('d6').split('goal=')[1].split()[0])
    except: return 0
def bump():
    try: return rd('d9')=='1'
    except: return False

def turn_by_time(direction, dur):
    drive(-42*direction, 42*direction)  # direction=1: increase heading side... whatever, consistent
    time.sleep(dur)
    stop()

def main():
    last_g=0; last_ping=0
    prev_sig=None; decreases=0
    last_n=0
    while True:
        now=time.time()
        if goal():
            stop()
            if now-last_g>8:
                last_g=now; tx('GOALFOUND from=beta (standing on goal)'); log('GOAL standing d5=%.3f'%sig['v'])
            time.sleep(0.3); continue
        if bump():
            drive(-45,-45); time.sleep(0.45)
            turn_by_time(random.choice([1,-1]), random.uniform(0.2,0.5))
            continue
        l=lidar()
        front=min(l[0], l[1]*1.3, l[15]*1.3)
        if front<0.26 or min(l[1],l[15])<0.15:
            # turn toward more open side
            left_open=max(l[13],l[14],l[12]); right_open=max(l[2],l[3],l[4])
            d=1 if right_open>left_open else -1
            drive(42*d, -42*d)
            t0=time.time()
            while time.time()-t0<4:
                l=lidar()
                if min(l[0],l[1]*1.2,l[15]*1.2)>0.45: break
                time.sleep(0.03)
            stop()
            continue
        # tumble logic
        if sig['n']>last_n:
            last_n=sig['n']
            v=sig['v']
            if prev_sig is not None:
                if v<prev_sig-0.005: decreases+=1
                elif v>prev_sig+0.005: decreases=0
                if decreases>=2:
                    decreases=0
                    d=random.choice([1,-1])
                    ang=random.uniform(0.6,1.6)
                    drive(42*d,-42*d); time.sleep(ang); stop()
                    log('tumble d5=%.3f'%v)
            prev_sig=v
        # cruise with light repulsion
        rdiag=min(l[2],l[3]); ldiag=min(l[13],l[14])
        steer = -13.0/max(rdiag,0.07) + 13.0/max(ldiag,0.07)
        steer=max(-35,min(35,steer))
        base=60 if front>0.55 else 38
        drive(int(base+steer), int(base-steer))
        if now-last_ping>5:
            last_ping=now; tx('PING from=beta homing d5=%.2f'%sig['v'])
            log('d5=%.3f front=%.2f'%(sig['v'],front))
        time.sleep(0.06)
main()
