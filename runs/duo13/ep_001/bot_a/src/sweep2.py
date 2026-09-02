import sys, time, math
sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a
r=R()
rx=[0.0,0.0]; prev=None
def odo():
    global prev
    e=r.enc(); h=r.heading()
    if e and e[0] is not None and e[1] is not None and prev and prev[0] is not None and prev[1] is not None and h is not None:
        d=((e[0]-prev[0])+(e[1]-prev[1]))/2.0*0.000503
        rx[0]+=d*math.cos(math.radians(h)); rx[1]+=d*math.sin(math.radians(h))
    if e: prev=e
def turnto(target,tol=4.0):
    for _ in range(60):
        h=r.heading()
        if h is None: time.sleep(0.05); continue
        e=wrap(target-h)
        if abs(e)<tol: r.motors(0,0); time.sleep(0.06); return
        v=int(max(-80,min(80,2.2*e)))
        r.motors(v,-v); time.sleep(0.07)
    r.motors(0,0)
def goto(tx,ty,maxt=4.0):
    t0=time.time()
    while time.time()-t0<maxt:
        st=r.status() or (0,0,0)
        if st[2]==1:
            r.motors(0,0); return 'HERE'
        rg=r.ranges()
        if rg is None: r.motors(0,0); time.sleep(0.05); continue
        f0=rg[0] if (rg[0] is not None and rg[0]>=0) else 9
        if f0<0.26:
            r.motors(0,0); return 'wall'
        if math.hypot(tx-rx[0],ty-rx[1])<0.10:
            r.motors(0,0); return 'ok'
        h=r.heading()
        if h is None: r.motors(0,0); time.sleep(0.05); continue
        ang=math.degrees(math.atan2(ty-rx[1],tx-rx[0]))
        err=wrap(ang-h)
        a=int(max(-40,min(95, 55+1.5*err)))
        b=int(max(-40,min(95, 55-1.5*err)))
        r.motors(a,b); time.sleep(0.09)
    r.motors(0,0); return 'to'
print("LAWNMOWER START",flush=True)
t0=time.time(); found=False
y=-0.66
row=0
while y<=0.66 and time.time()-t0<420:
    xs=[-0.66+i*0.22 for i in range(7)]
    if row%2: xs=xs[::-1]
    for xx in xs:
        st=r.status() or (0,0,0)
        d5=r.read(5,0.03); d0=r.read(0,0.03)
        if st[2]==1: found=True; break
        if d5=='1' or d0=='1': print(f"  beacon! at ({rx[0]:.2f},{rx[1]:.2f}) d0={d0} d5={d5}",flush=True)
        res=goto(xx,y)
        odo()
        if res=='HERE': found=True; break
    if found: break
    y+=0.22; row+=1
print("mower done found=",found,flush=True)
if found:
    print("*** HERE=1 — ON GOAL ***",flush=True)
    r.motors(0,0)
    while True:
        r.write(8,f"B2 ATGOAL I AM ON GOAL t={time.time():.0f}")
        time.sleep(1.0)
