import sys,time,math,json
sys.path.insert(0,"/bot/src")
from robot import speed,turn,stop,send
from nav import hd,sc,odo
from robust import rline

t0=time.time()
log=open("/memory/homer.log","a")
buf=[]  # (t, x, y, rss)
x=y=0.0; prev_o=None; prev_h=None
last_ping=0
phi=225.0  # initial guess (deg)
last_phi_change=0

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

def gradient():
    # LSQ fit rss ~ a*x+b*y over buffer (centered)
    n=len(buf)
    if n<30: return None
    mx=sum(b[1] for b in buf)/n; my=sum(b[2] for b in buf)/n
    mr=sum(b[3] for b in buf)/n
    sxx=sum((b[1]-mx)**2 for b in buf); syy=sum((b[2]-my)**2 for b in buf)
    sxy=sum((b[1]-mx)*(b[2]-my) for b in buf)
    if sxx<25 or syy<25: return None  # not enough spread
    sxr=sum((b[1]-mx)*(b[3]-mr) for b in buf)
    syr=sum((b[2]-my)*(b[3]-mr) for b in buf)
    # solve [sxx sxy; sxy syy] [a;b] = [sxr; syr]
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-6: return None
    a=(sxr*syy-syr*sxy)/det; b=(syr*sxx-sxr*sxy)/det
    return a,b

BUDGET=1200
steer_mode="gradient"
avoid_until=0
try:
    while time.time()-t0<BUDGET:
        now=time.time()-t0
        h=hd()
        if h is None: time.sleep(0.05); continue
        update_pose(h)
        s=sc()
        rss=read_rss()
        if rss is not None:
            buf.append((now,x,y,rss))
            if len(buf)>400: buf.pop(0)
        # ping every 15s
        if now-last_ping>15:
            last_ping=now
            send(f"A homing t={now:.0f} x={x:.0f} y={y:.0f} rss={rss}")
        # steering decision
        g=gradient()
        if g and (now-last_phi_change>4):
            ga=math.degrees(math.atan2(g[1],g[0]))
            mag=math.hypot(g[0],g[1])
            if mag>1e-7:
                phi=ga
        e=(phi-h+540)%360-180
        turn(max(-40,min(40,-e*2.0)))
        # obstacle avoidance override
        sp=3.0
        if s:
            f0=s[0] if s[0]>=0 else 3.2
            f1=s[1] if s[1]>=0 else 3.2
            f15=s[15] if s[15]>=0 else 3.2
            front=min(f0,f1,f15)
            if front<0.9:
                # pick most open beam among near-forward, bias by gradient dir
                best=None;bs=-1
                for k in range(16):
                    v=s[k] if s[k]>=0 else 3.2
                    a=k*22.5
                    if a>180: a-=360
                    if abs(a)<80:
                        sc_=v-0.3*abs(a)/90
                        if sc_>bs: bs=sc_;best=k
                if best is not None:
                    ta=best*22.5
                    if ta>180: ta-=360
                    phi=ta  # temporary avoidance heading
                    last_phi_change=now
            sp = 3 if front>1.2 else (1.6 if front>0.5 else 0.8)
        speed(sp)
        log.write(json.dumps({"t":round(now,1),"h":round(h,1),"x":round(x,1),"y":round(y,1),"rss":rss,"phi":round(phi,1)})+"\n")
        log.flush()
        time.sleep(0.12)
finally:
    stop(); log.close(); print("homer done",flush=True)
