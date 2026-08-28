import rob, time, statistics

CELL=0.5

def bng(): return float(rob.rd(10))
def serr(a,b): return (a-b+540)%360-180

def turn(delta, tol=5):
    tgt=(bng()+delta)%360
    t0=time.time()
    while time.time()-t0<25:
        e=serr(tgt,bng())
        if abs(e)<tol: rob.motors(0,0); return True
        s=max(8,min(55,abs(e)))
        if e>0: rob.motors(-s,s)
        else: rob.motors(s,-s)
        time.sleep(0.08)
    rob.motors(0,0); return False

def walls():
    Ls=[rob.lidar() for _ in range(3)]
    out=[]
    for i in (0,4,8,12):
        vs=[L[i] for L in Ls if L[i]>0]
        out.append(statistics.median(vs) if vs else 2.5)
    return out  # F,R,B,L

def center_step(spd=20):
    o0=rob.odo(); target=int(CELL*156)
    while rob.odo()-o0 < target:
        L=rob.lidar()
        F=L[0]
        if 0<F<0.22: break
        r_,l_=L[4],L[12]
        c=0
        if 0<r_<0.5 and 0<l_<0.5: c=(r_-l_)*30
        elif 0<r_<0.3: c=(r_-0.2)*40
        elif 0<l_<0.3: c=-(l_-0.2)*40
        c=max(-6,min(6,c))
        rob.motors(spd+c,spd-c)
        time.sleep(0.06)
    rob.motors(0,0)
    return rob.odo()-o0
