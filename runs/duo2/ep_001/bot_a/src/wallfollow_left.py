import time, math, sys, random
from robot import Robot

r=Robot()
MAXR=2.5
track=open('/memory/track.log','a')
track.write(f"# wf4-LEFTwall {time.time()}\n")
x,y=0.0,0.0
CMD2SPD=0.0124  # u/s per wheel cmd

def clean(s): return [MAXR if v<0 else min(v,MAXR) for v in s]

def check_goal():
    if r.goal():
        r.stop(); print("GOAL REACHED",flush=True)
        track.write(f"GOAL {x:.2f} {y:.2f}\n"); track.flush(); sys.exit(0)

t_last=time.time(); msg_t=0; log_t=0; detach_t=time.time()
scans=[]; stuck_n=0; cur_v=0
while True:
    check_goal()
    s=r.scan(); h=r.heading()
    if not s or h is None: time.sleep(0.1); continue
    s=clean(s)
    now=time.time(); dt=now-t_last; t_last=now
    scans.append((now,s)); scans=[e for e in scans if now-e[0]<6]
    if now-scans[0][0]>4.5:
        old=scans[0][1]
        diffs=[abs(a-b) for a,b in zip(old,s) if a<2.3 and b<2.3]
        if diffs and max(diffs)<0.12:
            stuck_n+=1
            track.write(f"FROZEN n={stuck_n}\n"); track.flush()
            r.stop(); time.sleep(0.2)
            if stuck_n%2==1: r.drive(-30,random.choice([-15,15]))
            else: r.drive(30,random.choice([-15,15]))
            time.sleep(1.5); r.stop(); scans=[]; continue
    if now-detach_t>150:
        detach_t=now
        track.write("DETACH\n"); track.flush()
        best=max(range(16), key=lambda i: min(s[i], s[(i+1)%16]*1.6+0.1, s[(i-1)%16]*1.6+0.1))
        tgt=(h+22.5*best)%360
        r.turn_to(tgt)
        t0=time.time()
        while time.time()-t0<8:
            s2=r.scan()
            if not s2: continue
            s2=clean(s2)
            if min(s2[0],s2[1]+0.12,s2[15]+0.12)<0.38: break
            r.drive(38,0); time.sleep(0.1)
        r.stop(); scans=[]; t_last=time.time(); continue
    front=min(s[0],s[1]+0.12,s[15]+0.12)
    right=s[4]; frd=s[2]
    blocked = front<0.38
    if blocked:
        # turn left (heading up? need away from wall) - turn toward more open side
        left_open=s[4]+s[3]+s[2]; right_open=s[12]+s[13]+s[14]
        wdir = 25 if left_open>=right_open else -25
        t0=time.time(); ok=False
        r.drive(0,0); time.sleep(0.1)
        while time.time()-t0<12:
            s2=r.scan()
            if not s2: continue
            s2=clean(s2)
            if s2[0]>0.7 and s2[1]>0.3 and s2[15]>0.3: ok=True; break
            r.drive(0,wdir); time.sleep(0.08)
        r.stop()
        if not ok:
            r.drive(-30,0); time.sleep(1.2); r.stop()
        scans=[]; stuck_n=0
        continue
    # right-wall follow: wall at beam 12 (which is heading-decreasing side? beam12 = h+270 = h-90)
    dr=min(right,0.71*frd)
    err=0.30-dr
    w=-err*35     # dr small -> err>0 -> w>0 -> heading up -> away from beam-12 side  (beam12 is at h-90)
    if right>0.9 and frd>1.0: w=14   # opening on right: turn toward it
    w=max(min(w,16),-16)
    v=38 if front>0.8 else 22
    r.drive(v,w)
    x+=CMD2SPD*v*dt*math.cos(math.radians(h)); y+=CMD2SPD*v*dt*math.sin(math.radians(h))
    if now-log_t>3:
        log_t=now
        track.write(f"W {x:.2f} {y:.2f} h={h:.0f} f={front:.2f} r={right:.2f} scan={','.join('%.2f'%v for v in s)}\n"); track.flush()
    if now-msg_t>25:
        msg_t=now; r.tx.write("hello")
        m=r.rx.last_line(0.2)
        if m: track.write("RX "+m+"\n"); track.flush()
    time.sleep(0.1)
