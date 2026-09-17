import time, math, os, sys
from drv import rd, wr
LOG=open('/bot/src/auto.log','a',buffering=1)
STATE=open('/bot/src/auto_state.csv','a',buffering=1)
def log(msg):
    t=time.time()
    LOG.write(f'[{t%100000:8.1f}] {msg}\n')
# beam angles: index i -> angle deg CCW from front (ASSUMPTION, to verify)
ANG=[i*22.5 for i in range(16)]
def norm180(a): 
    a=(a+180)%360-180
    return a
def read_beams():
    try:
        return [float(x) for x in rd('d5').split(',')]
    except: return None
def clean(b): return [6.0 if x<0 else x for x in b]
last_lap=None; last_goal=None; last_best=None; last_last=None
D4,D7=0,0
def emit(d4,d7):
    global D4,D7
    if d4!=D4 or d7!=D7:
        try:
            wr('d4',str(int(d4))); wr('d7',str(int(d7))); D4,D7=d4,d7
        except Exception as e: log(f'emit err {e}')
t0=time.time()
log('autopilot start')
cycle=0
while True:
    if os.path.exists('/bot/src/STOP'):
        emit(0,0); log('STOP file seen; halting'); break
    cycle+=1
    b=read_beams()
    if b is None:
        time.sleep(0.05); continue
    bc=clean(b)
    hd=0.0; sp=0.0; wz=0.0
    try: hd=float(rd('d3',0.02))
    except: pass
    try: sp=float(rd('d2',0.02))
    except: pass
    try: imu=[float(x) for x in rd('d0',0.02).split(',')]; wz=imu[2]
    except: pass
    d1=rd('d1',0.02); d6=rd('d6',0.02); d8=rd('d8',0.02)
    # d8 events
    try:
        parts=dict(kv.split('=') for kv in d8.split())
        lap=parts['lap']; goal=parts['goal']; best=parts['best']; lastt=parts['last']
        if lap!=last_lap: log(f'*** LAP {last_lap}->{lap} d8={d8}'); last_lap=lap
        if goal!=last_goal: log(f'*** GOAL {last_goal}->{goal} d8={d8}'); last_goal=goal
        if best!=last_best: log(f'*** BEST {last_best}->{best} d8={d8}'); last_best=best
        if lastt!=last_last: log(f'*** LAST {last_last}->{lastt} d8={d8}'); last_last=lastt
    except Exception as e:
        pass
    if d1=='1': log(f'EVENT d1=1 hd={hd:.0f} v={sp:.2f} d8={d8}')
    if d6=='1': pass  # collision is common, skip logging
    front=min(bc[15],bc[0],bc[1])
    left=min(bc[2],bc[3],bc[4],bc[5])
    right=min(bc[11],bc[12],bc[13],bc[14])
    # choose target ray: among all rays, score = clearance - angular penalty
    best_i=0; best_s=-1
    for i in range(16):
        c=min(bc[i],4.0)
        pen=abs(norm180(ANG[i]))/90.0*0.8
        # prefer slight right (race direction unknown); neutral
        s=c-pen
        if s>best_s: best_s=s; best_i=i
    tgt=ANG[best_i]
    # centering bias once channel detected
    corridor = bc[4]<1.0 and bc[12]<1.0 and bc[0]>2.0
    if corridor:
        err=bc[4]-bc[12]  # left vs right clearance
        steer_c=+err*60   # steer toward more open side? sign TBD
    else:
        steer_c=0.0
    steer = norm180(tgt)*1.4 + steer_c
    d7=max(-100,min(100,steer))
    if front<0.35:
        # emergency: reverse and swing nose toward best side
        d4=-45; d7=max(-100,min(100, -steer))
        mode='REV'
    else:
        if front>1.2: base=38
        elif front>0.7: base=28
        else: base=18
        d4=base; mode='FWD'
    emit(d4,d7)
    STATE.write(f'{time.time()-t0:.2f},{cycle},{hd:.1f},{sp:.2f},{wz:.2f},{d4},{d7},{mode},{";".join(f"{x:.2f}" for x in b)},{d8}\n')
    if cycle%20==0:
        log(f'c{cycle} hd={hd:.0f} v={sp:.2f} wz={wz:.2f} front={front:.2f} L={left:.2f} R={right:.2f} best_i={best_i} d4={d4} d7={d7} {mode} corr={corridor} d8={d8}')
    time.sleep(0.05)
