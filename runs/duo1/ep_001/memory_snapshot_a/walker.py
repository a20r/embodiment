import rob, time, math, random

def bng(): return float(rob.rd(10))
def serr(a,b): return (a-b+540)%360-180

def turn(delta, tol=5):
    tgt=(bng()+delta)%360
    t0=time.time()
    while time.time()-t0<20:
        e=serr(tgt,bng())
        if abs(e)<tol: rob.motors(0,0); return True
        s=max(8,min(55,abs(e)))
        if e>0: rob.motors(-s,s)
        else: rob.motors(s,-s)
        time.sleep(0.08)
    rob.motors(0,0); return False

def align():
    """align to corridor walls using side diagonals"""
    for _ in range(6):
        L=rob.lidar()
        err=None
        if 0<L[4]<0.35 and 0<L[3] and 0<L[5]:
            err=(L[5]-L[3])
        elif 0<L[12]<0.35 and 0<L[11] and 0<L[13]:
            err=(L[11]-L[13])
        if err is None or abs(err)<0.03: return
        d=max(-15,min(15,err*120))
        turn(d, tol=4)

def step(spd=22):
    """one cell forward w/ centering; return 'ok'|'wall'"""
    o0=rob.odo(); target=78
    while rob.odo()-o0 < target:
        L=rob.lidar()
        if 0<L[0]<0.20: rob.motors(0,0); return 'wall'
        r_,l_=L[4],L[12]
        c=0.0
        if 0<r_<0.45 and 0<l_<0.45: c=(r_-l_)*35
        elif 0<r_<0.28: c=(r_-0.18)*45
        elif 0<l_<0.28: c=-(l_-0.18)*45
        # parallelism
        if 0<r_<0.3 and L[3]>0 and L[5]>0: c+= (L[5]-L[3])*18
        if 0<l_<0.3 and L[11]>0 and L[13]>0: c+= (L[11]-L[13])*18
        c=max(-7,min(7,c))
        rob.motors(spd+c,spd-c)
        time.sleep(0.06)
    rob.motors(0,0); return 'ok'

def med(i, Ls): 
    vs=[L[i] for L in Ls if L[i]>0]
    vs.sort()
    return vs[len(vs)//2] if vs else 2.5

def look():
    Ls=[rob.lidar() for _ in range(3)]
    return med(0,Ls), med(4,Ls), med(12,Ls), med(8,Ls)  # F,R,L,B

def main():
    t0=time.time()
    last_dirs=[]
    while time.time()-t0<3300:
        if rob.goal(): print("GOAL!!!", flush=True); return
        align()
        F,R,Lt,B=look()
        b=bng()
        opts=[]
        # weight: openness + goal-bias
        def w(dirdelta, dist):
            if dist<0.42: return 0
            gb=abs(serr(b, {0:0,90:-90,-90:90}[dirdelta] if False else 0))
            # bias toward smaller |bearing after turn|: bearing reading after turning by delta = b+delta
            nb=abs(serr((b+dirdelta)%360,0))
            return (1.6 if dirdelta==0 else 1.0) + dist*0.25
        cand=[(0,F),(90,R),(-90,Lt)]
        # NOTE turn(+90) increases reading; beam4(right) is at +90 reading direction
        weights=[]
        for dd,dist in cand:
            weights.append(w(dd,dist))
        if sum(weights)<=0:
            # dead end: turn around
            print(f"deadend F={F:.2f} R={R:.2f} L={Lt:.2f} b={b:.0f}", flush=True)
            turn(180); continue
        dd=random.choices([c[0] for c in cand],weights=weights)[0]
        if dd: turn(dd)
        r=step()
        print(f"b={b:.0f} F={F:.2f} R={R:.2f} L={Lt:.2f} -> turn {dd} step {r}", flush=True)
    print("timeout", flush=True)

if __name__=="__main__":
    main()
