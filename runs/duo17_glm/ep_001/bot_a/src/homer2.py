import sys,time,math,json,statistics
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop,send
from nav import hd,sc,odo
from robust import rline

t0=time.time()
log=open("/memory/homer.log","a")
x=y=0.0; prev_o=None; prev_h=None
last_ping=-99

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

def sample_rss(secs):
    vals=[]; t0=time.time()
    while time.time()-t0<secs:
        v=read_rss()
        if v is not None: vals.append(v)
        time.sleep(0.1)
    return statistics.mean(vals) if vals else None

def turn_to(target,tol=5,maxt=8):
    t0=time.time()
    while time.time()-t0<maxt:
        h=hd()
        if h is None: continue
        e=(target-h+540)%360-180
        if abs(e)<tol: turn(0); return h
        turn(max(-40,min(40,-e*2.0)))
        time.sleep(0.05)
    turn(0); return hd()

def probe(dir_deg, secs=18):
    """drive toward dir_deg with avoidance; return (mean_rss_last, dist)"""
    turn_to(dir_deg)
    time.sleep(0.3)
    o0=odo()
    vals=[]
    t0=time.time()
    while time.time()-t0<secs:
        h=hd()
        if h is None: continue
        update_pose(h)
        s=sc()
        # steer: hold dir unless blocked
        e=(dir_deg-h+540)%360-180
        turn(max(-35,min(35,-e*2.0)))
        sp=3.0
        if s:
            f0=s[0] if s[0]>=0 else 3.2
            f1=s[1] if s[1]>=0 else 3.2
            f15=s[15] if s[15]>=0 else 3.2
            front=min(f0,f1,f15)
            if front<0.5:
                best=None;bs=-1
                for k in range(16):
                    v=s[k] if s[k]>=0 else 3.2
                    a=k*22.5
                    if a>180: a-=360
                    if abs(a)<85:
                        sc_=v-0.25*abs(a)/90
                        if sc_>bs: bs=sc_;best=k
                if best is not None:
                    ta=best*22.5
                    if ta>180: ta-=360
                    turn_to(ta,tol=8,maxt=2)
            sp = 3 if front>1.0 else (1.5 if front>0.45 else 0.7)
        speed(sp)
        if time.time()-t0>secs-5:
            v=read_rss()
            if v is not None: vals.append(v)
        log.write(json.dumps({"t":round(time.time()-t0+T0,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":read_rss(),"probe":dir_deg})+"\n"); log.flush()
        time.sleep(0.12)
    speed(0)
    o1=odo()
    dist=abs(o1-o0) if o1 and o0 else 0
    m=statistics.mean(vals) if vals else None
    return m,dist

T0=time.time()-t0
phi=225.0
cycle=0
try:
    while time.time()-t0<1800:
        cycle+=1
        now=time.time()-t0
        if now-last_ping>20:
            last_ping=now
            send(f"A probe cycle={cycle} p=({x:.0f},{y:.0f}) phi={phi:.0f}")
        b0=sample_rss(3)
        m1,d1=probe(phi)
        m2,d2=probe(phi+90)
        g1=(m1-b0) if (m1 is not None and b0 is not None) else None
        g2=(m2-m1) if (m2 is not None and m1 is not None) else None
        gx=gy=0.0
        if g1 is not None and d1>30: gx=g1/d1*math.cos(math.radians(phi)); gy=g1/d1*math.sin(math.radians(phi))
        if g2 is not None and d2>30: gx+=g2/d2*math.cos(math.radians(phi+90)); gy+=g2/d2*math.sin(math.radians(phi+90))
        if math.hypot(gx,gy)>1e-9:
            phi=math.degrees(math.atan2(gy,gx))
        print(f"cycle={cycle} b0={b0} m1={m1} d1={d1:.0f} m2={m2} d2={d2:.0f} grad=({gx:.2e},{gy:.2e}) phi={phi:.0f} pos=({x:.0f},{y:.0f})",flush=True)
        with open("/memory/homer_status.txt","w") as f:
            f.write(f"cycle={cycle} phi={phi:.0f} pos=({x:.0f},{y:.0f}) b0={b0} m1={m1} m2={m2}\n")
finally:
    stop(); print("homer2 done",flush=True)
