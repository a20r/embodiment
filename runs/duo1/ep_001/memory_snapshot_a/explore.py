import rob, walker, time, math, json, random

def ad(x): return (x+180)%360-180
bng=walker.bng

class Loc:
    def __init__(s, D0):
        s.D=D0; s.phi=0.0
    def step(s, ds, b):
        br=math.radians(b)
        Dn=math.sqrt(max(0.01, s.D*s.D + ds*ds - 2*s.D*ds*math.cos(br)))
        dphi=math.degrees(math.asin(max(-1,min(1, ds*math.sin(br)/max(0.05,Dn)))))
        s.D=Dn; s.phi=(s.phi+dphi)%360
    def xy(s):
        return s.D*math.cos(math.radians(s.phi)), s.D*math.sin(math.radians(s.phi))

def measure_D():
    # rotate to b=90, drive short, use bearing change; lidar-front unavailable generally -> use odo
    walker.turn(ad(90-bng()))
    b0=bng(); o0=rob.odo()
    rob.motors(14,14)
    while rob.odo()-o0<50:
        L=rob.lidar()
        if 0<L[0]<0.25: break
        time.sleep(0.04)
    rob.motors(0,0)
    ds=(rob.odo()-o0)/156.0
    b1=bng()
    db=abs(math.radians(ad(b1-b0)))
    if db<0.02: return None
    D=ds*math.sin(math.radians((b0+b1)/2))/math.tan(db)
    return abs(D)

def probe():
    rob.wr(6,"hello")
    out=''
    for _ in range(25):
        s=rob.rd(5,tries=1)
        if s: out+=s
    return out

def main():
    t0=time.time()
    D0=None
    while D0 is None:
        D0=measure_D()
    loc=Loc(D0)
    print(f"D0={D0:.2f}", flush=True)
    visits={}
    probed=set()
    while time.time()-t0<3300:
        if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); return
        walker.align()
        F,R,Lt,B=walker.look()
        b=bng()
        x,y=loc.xy()
        ck=(round(x/0.45), round(y/0.45))
        visits[ck]=visits.get(ck,0)+1
        r=probe()
        if r: print(f"RADIOHIT {r!r} at {ck} D={loc.D:.2f} phi={loc.phi:.0f} b={b:.0f}", flush=True)
        # candidate dirs
        theta=(loc.phi+180-b)%360
        opts=[]
        for dd,dist in ((0,F),(90,R),(-90,Lt),(180,B)):
            if dist<0.38: continue
            th=(theta-dd)%360
            tx,ty=x+0.45*math.cos(math.radians(th)), y+0.45*math.sin(math.radians(th))
            tk=(round(tx/0.45), round(ty/0.45))
            v=visits.get(tk,0)
            pen = 0 if dd==0 else (0.4 if dd!=180 else 1.2)
            opts.append((v+pen+random.random()*0.3, dd, dist))
        if not opts:
            walker.turn(180); continue
        opts.sort()
        _,dd,dist=opts[0]
        if dd: walker.turn(dd)
        # step with ds/b tracking
        b0=bng(); o0=rob.odo(); F0=rob.lidar()[0]
        st=walker.step(spd=24)
        b1=bng(); F1=rob.lidar()[0]
        ds=(rob.odo()-o0)/156.0
        if F0>0.3 and F1>0 and 0<F0-F1<0.8: ds=F0-F1
        bm=(b0+ad(b1-b0)/2)%360
        loc.step(ds, bm)
        # D re-estimation from bearing change if straight & significant
        db=ad(b1-b0)
        if abs(db)>6 and ds>0.15:
            Dest=abs(ds*math.sin(math.radians(bm))/math.tan(math.radians(abs(db))))
            if 0.1<Dest<4.0:
                loc.D=0.6*loc.D+0.4*Dest
        if random.random()<0.12:
            print(f"t={time.time()-t0:.0f} D={loc.D:.2f} phi={loc.phi:.0f} cells={len(visits)} b={b1:.0f}", flush=True)
        # periodic save
        if len(visits)%25==0:
            json.dump({str(k):v for k,v in visits.items()}, open('/memory/cells.json','w'))
    print("end cells=%d"%len(visits), flush=True)
    json.dump({str(k):v for k,v in visits.items()}, open('/memory/cells.json','w'))

if __name__=="__main__":
    main()
