import math, sys, re, time
def load(win=75.0):
    pings=[]
    for line in open('/memory/rx.log'):
        m=re.search(r'PING x=(-?[\d.]+) y=(-?[\d.]+) t=(\d+)',line)
        if m:
            pings.append((float(m.group(3)),float(m.group(1)),float(m.group(2))))
    rows=[]
    for line in open('/memory/tri3.csv'):
        p=line.strip().split(',')
        if len(p)>=5:
            try: rows.append((float(p[0]),float(p[1]),float(p[2]),float(p[3]),float(p[4])))
            except: pass
    if not rows or not pings: return []
    now=time.time()
    pings=[p for p in pings if now-p[0]<win]
    out=[]
    for (pt,qx,qy) in pings:
        best=min(rows,key=lambda r:abs(r[0]-pt))
        if abs(best[0]-pt)<0.8 and best[4]>0:
            out.append((best[1],best[2],best[3],best[4],qx,qy,pt))
    # make window-relative: subtract first sample pose
    if out:
        x0,y0=out[0][0],out[0][1]
        out=[(x-x0,y-y0,h,d,qx,qy,t) for (x,y,h,d,qx,qy,t) in out]
    return out
def fit(ev):
    best=None
    n=len(ev)
    for s in [x*0.0005 for x in range(8,26)]:
        for thd in range(0,360,3):
            th=math.radians(thd); c,sn=math.cos(th),math.sin(th)
            for mir in (False,True):
                sx=sy=0
                for (x,y,h,d,qx,qy,t) in ev:
                    px,py=(y,x) if mir else (x,y)
                    sx+=qx-s*(c*px-sn*py); sy+=qy-s*(sn*px+c*py)
                tx_,ty_=sx/n,sy/n
                r=0
                for (x,y,h,d,qx,qy,t) in ev:
                    px,py=(y,x) if mir else (x,y)
                    wx=s*(c*px-sn*py)+tx_; wy=s*(sn*px+c*py)+ty_
                    r+=(math.hypot(wx-qx,wy-qy)-d)**2
                rms=math.sqrt(r/n)
                if best is None or rms<best[0]: best=(rms,s,thd,tx_,ty_,mir)
    return best
if __name__=='__main__':
    win=float(sys.argv[1]) if len(sys.argv)>1 else 75.0
    ev=load(win)
    print(f"{len(ev)} paired samples in {win}s window")
    if len(ev)<6: print("insufficient"); sys.exit()
    b=fit(ev)
    print(f"BEST rms={b[0]:.3f} s={b[1]:.4f} th={b[2]} T=({b[3]:.2f},{b[4]:.2f}) mir={b[5]}")
