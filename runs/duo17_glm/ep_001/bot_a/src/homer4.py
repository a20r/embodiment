import sys,time,math,json,statistics
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop,send
from nav import hd,sc,odo
from robust import rline

t0=time.time()
log=open("/memory/homer.log","a")
x=y=0.0; prev_o=None; prev_h=None
last_ping=-99
phi=112.0   # toward the eastern opening initially

def read_rss():
    v=rline("d11",0.2)
    try: return float(v)
    except: return None

def update_pose(h):
    global x,y,prev_o,prev_h
    o=odo()
    if o is not None and prev_o is not None:
        dd=o-prev_o; hh=h if prev_h is None else (h+prev_h)/2
        x+=dd*math.cos(math.radians(hh)); y+=dd*math.sin(math.radians(hh))
    if o is not None: prev_o=o
    prev_h=h

SEG=60.0     # seconds per test segment
BUDGET=2400
seg_start=time.time()-t0
seg_rss=[]
seg_prev=None   # (phi, mean_rss, dist, x, y)
o_seg0=odo()
try:
    while time.time()-t0<BUDGET:
        now=time.time()-t0
        h=hd()
        if h is None: time.sleep(0.05); continue
        update_pose(h)
        s=sc()
        rss=read_rss()
        if rss is not None: seg_rss.append(rss)
        if now-last_ping>20:
            last_ping=now
            send(f"A p=({x:.0f},{y:.0f}) phi={phi:.0f} rss={rss}")
        # segment transition
        if now-seg_start>=SEG:
            o1=odo()
            dist=abs(o1-o_seg0) if (o1 and o_seg0 is not None) else 0
            o_seg0=o1
            if seg_rss:
                m=statistics.mean(seg_rss)
                if seg_prev is not None:
                    pphi,pm,pdist=seg_prev
                    # compare this segment's rss trend to previous
                    dm=m-pm
                    # gradient along this segment's phi:
                    # if rss improved while driving at pphi, keep pphi; else rotate
                    if dm>0.004 and dist>200:
                        phi=pphi  # improving: keep
                    elif dm<-0.004 and dist>200:
                        phi=(pphi+90)%360  # worsening: turn 90 (choose CW first)
                    seg_prev=(phi,m,dist)
                else:
                    seg_prev=(phi,m,dist)
                print(f"t={now:.0f} seg ended phi={phi:.0f} m={m:.4f} dist={dist:.0f} pos=({x:.0f},{y:.0f})",flush=True)
            seg_rss=[]
            seg_start=now
        # gap-seeking with phi bias
        if s:
            best=None;bs=-1
            for k in range(16):
                v=s[k] if s[k]>=0 else 3.2
                a=k*22.5
                if a>180: a-=360
                # beam world dir = h + a ; angle diff from phi
                d=(h+a-phi+540)%360-180
                bias=0.35+0.65*max(0.0,math.cos(math.radians(d)))
                sc_=v*bias
                if sc_>bs: bs=sc_;best=k
            ta=best*22.5
            if ta>180: ta-=360
            e=(ta-h+540)%360-180
            turn(max(-40,min(40,-e*1.6)))
            fv=[s[15] if s[15]>=0 else 3.2, s[0] if s[0]>=0 else 3.2, s[1] if s[1]>=0 else 3.2, s[2] if s[2]>=0 else 3.2]
            front=min(fv)
            vmax=max(v if v>=0 else 3.2 for v in s)
            sp = 4 if (vmax>1.5 and front>1.0) else (2.5 if front>0.5 else 1.0)
            if front<0.2: sp=0.5
            speed(sp)
        log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"phi":round(phi,1)})+"\n"); log.flush()
        time.sleep(0.12)
finally:
    stop(); print("homer4 done",flush=True)
