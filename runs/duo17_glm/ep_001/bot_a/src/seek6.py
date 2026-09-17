import sys,time,math,json,statistics
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop,send
from nav import hd,sc,odo
from robust import rline

t0=time.time()
log=open("/memory/seek.log","a")
x=y=0.0; prev_o=None; prev_h=None
last_ping=-99
phi=90.0
SEG=90.0
seg_start=0.0
seg_rss=[]; seg_o0=None; seg_h=[]
seg_prev=None
rot_dir=1
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

def unstick():
    speed(-2); time.sleep(1.3); speed(0)
    s=sc()
    if s:
        best=None;bs=-1
        for k in range(16):
            v=s[k] if s[k]>=0 else 3.2
            if v>bs: bs=v;best=k
        ta=best*22.5
        if ta>180: ta-=360
        t0u=time.time()
        while time.time()-t0u<6:
            h=hd()
            if h is None: continue
            e=(ta+540)%360-180
            if abs(e)<10: break
            turn(max(-35,min(35,-e*2)))
            time.sleep(0.05)
        turn(0)

BUDGET=3000
try:
    while time.time()-t0<BUDGET:
        now=time.time()-t0
        h=hd()
        if h is None: time.sleep(0.05); continue
        update_pose(h)
        s=sc()
        rss=read_rss()
        if seg_o0 is None: seg_o0=odo()
        if rss is not None: seg_rss.append(rss)
        if h is not None: seg_h.append(h)
        if now-last_ping>25:
            last_ping=now
            send(f"A p=({x:.0f},{y:.0f}) rss={rss}")
        if now-seg_start>=SEG:
            o1=odo()
            dist=abs(o1-seg_o0) if (o1 is not None and seg_o0 is not None) else 0
            seg_o0=o1
            if seg_rss and seg_h:
                m=statistics.mean(seg_rss); mh=statistics.mean(seg_h)
                if seg_prev is not None:
                    pm,pmh=seg_prev
                    dm=m-pm
                    if dist<120: unstick(); phi=(phi+45*rot_dir)%360
                    elif dist>250:
                        if dm>0.005: pass  # keep phi
                        elif dm<-0.005: phi=(phi+90*rot_dir)%360; rot_dir*=-1
                        else: phi=(phi+90*rot_dir)%360  # flat: rotate
                    print(f"t={now:.0f} seg m={m:.4f} prev={pm:.4f} dm={dm:+.4f} dist={dist:.0f} mh={mh:.0f} phi={phi:.0f} pos=({x:.0f},{y:.0f})",flush=True)
                seg_prev=(m,mh)
            seg_rss=[]; seg_h=[]; seg_start=now
        if s:
            best=None;bs=-1
            for k in range(16):
                v=s[k] if s[k]>=0 else 3.2
                a=k*22.5
                if a>180: a-=360
                d=(h+a-phi+540)%360-180
                bias=0.35+0.65*max(0.0,math.cos(math.radians(d)))
                sc_=v*bias
                if sc_>bs: bs=sc_;best=k
            ta=best*22.5
            if ta>180: ta-=360
            e=(ta+540)%360-180   # RELATIVE target angle (bug fixed)
            turn(max(-40,min(40,-e*1.6)))
            fv=[s[15] if s[15]>=0 else 3.2, s[0] if s[0]>=0 else 3.2, s[1] if s[1]>=0 else 3.2, s[2] if s[2]>=0 else 3.2]
            front=min(fv)
            vmax=max(v if v>=0 else 3.2 for v in s)
            sp = 4 if (vmax>1.5 and front>1.0) else (2.5 if front>0.5 else 1.0)
            if front<0.2: sp=0.5
            speed(sp)
        log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"phi":round(phi,1)})+"\n"); log.flush()
        time.sleep(0.1)
finally:
    stop(); print("seek6 done",flush=True)
