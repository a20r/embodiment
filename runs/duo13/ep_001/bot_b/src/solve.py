import math, sys
def load(path='/memory/tri.csv'):
    # S rows: S,time,px,py ; preceding T row gives my local pose+d11
    events=[]
    lastT=None
    for line in open(path):
        p=line.strip().split(',')
        if not p or not p[0]: continue
        if p[0]=='T':
            # T,time,x,y,h,d11,rx
            try: lastT=(float(p[1]),float(p[2]),float(p[3]),float(p[4]),float(p[5]))
            except: lastT=None
        elif p[0]=='S' and lastT is not None:
            try: events.append((lastT[1],lastT[2],lastT[3],lastT[4],float(p[2]),float(p[3])))
            except: pass
            lastT=None
    return events
def fit(events, mirror=False):
    # unknowns: T(2), theta, s ; P_w = T + s*R(theta)*P_l
    best=None
    import itertools
    for s in [x*0.0005 for x in range(6,25)]:
        for th in range(0,360,5):
            th_=math.radians(th)
            c,sn=math.cos(th_),math.sin(th_)
            # R = [[c,-sn],[sn,c]] (rotate CCW)
            res=[]
            Ts=[[],[]]
            for (x,y,h,d11,qx,qy) in events:
                px,py=(y,x) if mirror else (x,y)
                wx=s*(c*px - sn*py); wy=s*(sn*px + c*py)
                Ts[0].append(qx-wx); Ts[1].append(qy-wy)
            tx=sum(Ts[0])/len(Ts[0]); ty=sum(Ts[1])/len(Ts[1])
            # refine T by coarse grid
            for _ in range(2):
                bestr=1e9; bt=(tx,ty)
                for dx in [-0.15,-0.075,0,0.075,0.15]:
                    for dy in [-0.15,-0.075,0,0.075,0.15]:
                        r=0
                        for (x,y,h,d11,qx,qy) in events:
                            px,py=(y,x) if mirror else (x,y)
                            wx=s*(c*px - sn*py)+tx+dx; wy=s*(sn*px + c*py)+ty+dy
                            d=math.hypot(wx-qx,wy-qy)-d11
                            r+=d*d
                        if r<bestr: bestr=r; bt=(tx+dx,ty+dy)
                tx,ty=bt
            r=0
            for (x,y,h,d11,qx,qy) in events:
                px,py=(y,x) if mirror else (x,y)
                wx=s*(c*px - sn*py)+tx; wy=s*(sn*px + c*py)+ty
                d=math.hypot(wx-qx,wy-qy)-d11
                r+=d*d
            rms=math.sqrt(r/len(events))
            if best is None or rms<best[0]:
                best=(rms,s,th,tx,ty,mirror)
    return best
if __name__=='__main__':
    ev=load()
    print(f"{len(ev)} events")
    if len(ev)<4:
        print("need more data"); sys.exit()
    b1=fit(ev,False); b2=fit(ev,True)
    for name,b in [('direct',b1),('mirror',b2)]:
        if b: print(f"{name}: rms={b[0]:.3f} s={b[1]:.4f} th={b[2]} T=({b[3]:.2f},{b[4]:.2f})")
    b=min([x for x in (b1,b2) if x])
    print("BEST:",b)
