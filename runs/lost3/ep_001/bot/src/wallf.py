# v7 right-wall follower, island detection via net rotation
import time, math, random
def rd(d):
    try:
        with open(f'/dev/robot/d{d}') as f: return f.readline().strip()
    except Exception: return ''
def wr(d,s):
    with open(f'/dev/robot/d{d}','w') as f: f.write(str(s)+'\n')
def lidar_med(n=2):
    scans=[]
    for _ in range(n+2):
        s=rd(4)
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: scans.append(v)
        except: pass
        if len(scans)>=n: break
    out=[]
    for i in range(16):
        vals=sorted(x[i] for x in scans if x[i]>=0)
        out.append(vals[len(vals)//2] if vals else 2.0)
    return out
def heading(n=3):
    vals=[]
    for _ in range(n+2):
        try: vals.append(float(rd(1)))
        except: pass
        if len(vals)>=n: break
    vals.sort(); return vals[len(vals)//2] if vals else 0.0
def stop(): wr(5,0); wr(6,0)
log=open('/memory/telemetry.csv','a')
def LOG(*a):
    log.write(','.join(str(x) for x in a)+'\n'); log.flush()

t0=time.time()
LOG('RESTART_V7', time.strftime('%H:%M:%S'))
prev_h=heading(); net=0.0
prev_scan=None; still=0
def goal_check():
    s0=rd(0)
    if 'goal=1' in s0:
        stop(); LOG('GOAL',s0); print('GOAL',s0); return True
    return False
try:
    while True:
        if goal_check(): break
        scan=lidar_med(2)
        h=heading()
        d=(h-prev_h+540)%360-180
        net+=d; prev_h=h
        bump=rd(2)=='1'
        LOG(round(time.time()-t0,1),'V7',round(h,1),round(net,0),int(bump),';'.join(f'{v:.2f}' for v in scan))
        if prev_scan and sum(abs(a-b) for a,b in zip(scan,prev_scan))<0.6:
            still+=1
        else: still=0
        prev_scan=scan
        front=min(scan[0],scan[15])
        right=min(scan[12],scan[13])
        rfront=scan[14]
        if net<=-355:
            # looped an island clockwise: detach, cross open space
            LOG('ISLAND_JUMP')
            net=0; prev_scan=None
            k=max(range(16),key=lambda i:min(scan[i],scan[(i+1)%16],scan[(i-1)%16]))
            # turn to k
            tgt=(h+22.5*k)%360
            for _ in range(50):
                cur=heading()
                err=(tgt-cur+540)%360-180
                if abs(err)<8: break
                r=max(15,min(40,abs(err)*.8))
                wr(5,-r if err>0 else r); wr(6,r if err>0 else -r)
                time.sleep(0.12)
            prev_h=heading()
            for _ in range(20):
                sc=lidar_med(1)
                if rd(2)=='1' or min(sc[0],sc[1],sc[15])<0.25: break
                wr(5,45);wr(6,45); time.sleep(0.4)
            stop(); continue
        if bump or still>=4:
            LOG('ESC','b' if bump else 's')
            still=0; prev_scan=None
            wr(5,-30);wr(6,-30); time.sleep(1.0); stop()
            # turn left 90ish
            wr(5,-22);wr(6,22); time.sleep(random.uniform(1.8,2.6)); stop()
            prev_h=heading()
            continue
        if front<0.3:
            wr(5,-24);wr(6,24); time.sleep(0.35)   # turn left
            continue
        if right>0.85 and rfront>0.85:
            wr(5,32);wr(6,15); time.sleep(0.4)      # arc right to find wall
            continue
        err=right-0.33
        steer=max(-9,min(9,err*28))
        if rfront<0.28: steer=-7
        b=int(max(28,min(70,50*front)))
        wr(5,int(b+steer)); wr(6,int(b-steer))
        time.sleep(0.4)
finally:
    stop(); LOG('END_V7',round(time.time()-t0,1))
