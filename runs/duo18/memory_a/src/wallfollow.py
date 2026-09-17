#!/usr/bin/env python3
# Left-wall follower with pose tracking, radio broadcast, and status monitoring.
import json, time, math, sys
sys.path.insert(0,'/bot/src')
from rio import write, tx
from ctl import st, scan, hd, angdiff, load_pose, save_pose, stop, TPU, HeadingFilter
LOG='/tmp/wf.log'
def log(msg):
    with open(LOG,'a') as f: f.write(f"{time.time():.1f} {msg}\n")
C22=math.cos(math.radians(22.5))
def run(duration=120, side='left', D=0.26):
    hf=HeadingFilter(); pose=load_pose(); s=st()
    lp,rp=int(s['d9']),int(s['d6'])
    t0=time.time(); lastlog=0; lasttx=0; back_until=0; turn_until=0; rxseen=0
    try: rxseen=len(open('/tmp/rx.log').read().splitlines())
    except: pass
    sgn = 1 if side=='left' else -1   # left wall: beams 12..15 are on the wall side
    while time.time()-t0<duration:
        s=st(); h=hf.upd(hd(s))
        try: l=int(s['d9']); r=int(s['d6']); d11=float(s['d11'])
        except: time.sleep(0.02); continue
        d=((l-lp)+(r-rp))/2*TPU; lp,rp=l,r
        if h is None: time.sleep(0.02); continue
        pose['x']+=d*math.sin(math.radians(h)); pose['y']+=d*math.cos(math.radians(h))
        st3=s.get('d3','')
        if 'goal=1' in st3 or 'here=1' in st3:
            stop(); save_pose(pose); log(f"STATUS {st3} pose=({pose['x']:.2f},{pose['y']:.2f})"); return 'status'
        try:
            rl=open('/tmp/rx.log').read().splitlines(); n=len(rl)
            if n>rxseen:
                new=rl[rxseen:]; rxseen=n
                for m in new: log("RX: "+m[:160])
                if any('B->A' in m for m in new): stop(); save_pose(pose); log("direct reply, stopping"); return 'rx'
        except: pass
        sc=scan(s); sc=[v if v>=0 else 3.0 for v in sc]
        if side=='right': sc=[sc[0]]+sc[1:][::-1]   # mirror so that logic is 'left'
        now=time.time()
        if now<back_until:
            write('d1','-70'); write('d7','-70'); time.sleep(0.05); continue
        if now<turn_until:
            write('d1',str(45*sgn)); write('d7',str(-45*sgn)); time.sleep(0.05); continue
        bump = s.get('d0')!='0' or s.get('d5')!='0'
        if bump or min(sc)<0.09:
            back_until=now+0.5; turn_until=now+0.9; log(f"escape bump={bump} min={min(sc):.2f}@{sc.index(min(sc))}"); continue
        front=min(sc[0], sc[1]/C22, sc[15]/C22)
        left=min(sc[12], sc[13]*C22, sc[11]*C22)
        fl=sc[14]  # 45 deg front-left
        if front<0.30:
            # blocked: rotate right (away from wall side)
            write('d1',str(50*sgn)); write('d7',str(-50*sgn)); mode='blocked'
        elif left>0.55 and fl>0.5:
            # lost the wall: arc left
            v=70; corr=-35*sgn
            write('d1',str(v+corr)); write('d7',str(v-corr)); mode='seek'
        else:
            err=(D-left)  # positive if too close -> turn right (left wheel faster)
            corr=err*250
            if fl<0.30: corr+=(0.30-fl)*300
            corr=max(-45,min(45,corr))*sgn
            v=max(60,min(170,(front-0.28)*350))
            write('d1',str(v+corr)); write('d7',str(v-corr)); mode='follow'
        if now-lastlog>3:
            lastlog=now; save_pose(pose)
            log(f"pose=({pose['x']:.2f},{pose['y']:.2f}) hd={h:.0f} d11={d11:.3f} {mode} front={front:.2f} left={left:.2f} fl={fl:.2f} {st3}")
        if now-lasttx>20:
            lasttx=now; tx(f"Robot A here, pose ({pose['x']:.1f},{pose['y']:.1f}). Reply if you hear me.")
        time.sleep(0.05)
    stop(); save_pose(pose); log("done"); return 'done'
if __name__=="__main__":
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 120
    side=sys.argv[2] if len(sys.argv)>2 else 'left'
    print(run(dur,side))
