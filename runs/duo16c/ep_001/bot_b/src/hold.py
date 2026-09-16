import math, subprocess, time, os, json, collections
def wr(p,v):
    for att in range(3):
        try:
            fd=os.open(f'/dev/robot/{p}',os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd,f'{int(v)}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.03)
def rd(p,tries=4):
    for _ in range(tries):
        r=subprocess.run(['timeout','0.3','cat',f'/dev/robot/{p}'],capture_output=True,text=True)
        if r.stdout.strip(): return r.stdout.strip()
        time.sleep(0.04)
    return '0'
def drive(a,b,dur,period=0.08):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(period)
def scan():
    s=rd('d2')
    return [(float(t.split(',')[0]), float(t.split(',')[1])) for t in s.split(';') if t]
def env_pts():
    pts=scan()
    return [(x,y) for x,y in pts if math.hypot(x,y)>=0.12 and not (math.hypot(x,y)<0.55 and not (-50<=math.degrees(math.atan2(y,x))<=35))]
def circle_center(env):
    n=len(env)
    Sxx=sum(x*x for x,y in env); Syy=sum(y*y for x,y in env); Sxy=sum(x*y for x,y in env)
    Sx=sum(x for x,y in env); Sy=sum(y for x,y in env)
    Sxz=sum(x*(x*x+y*y) for x,y in env); Syz=sum(y*(x*x+y*y) for x,y in env); Sz=sum(x*x+y*y for x,y in env)
    A=[[Sxx,Sxy,Sx],[Sxy,Syy,Sy],[Sx,Sy,n]]; B=[Sxz,Syz,Sz]
    for i in range(3):
        piv=A[i][i]
        for j in range(i+1,3):
            f=A[j][i]/piv
            for k in range(3): A[j][k]-=f*A[i][k]
            B[j]-=f*B[i]
    X=[0,0,0]
    for i in (2,1,0):
        X[i]=(B[i]-sum(A[i][k]*X[k] for k in range(i+1,3)))/A[i][i]
    a_,b_,c_=X
    cx,cy=a_/2,b_/2
    r=math.sqrt(max(c_+cx*cx+cy*cy,0))
    return cx,cy,r
log=open('/tmp/hold.log','a',buffering=1)
# 1) locate center
env=env_pts()
cx,cy,R=circle_center(env)
dist=math.hypot(cx,cy); az=math.degrees(math.atan2(cy,cx))
log.write(f'center robot-frame ({cx:.2f},{cy:.2f}) R={R:.2f} dist={dist:.2f} az={az:.0f}\n')
# 2) turn toward center bearing (closed loop, patient)
H=0.0; h=float(rd('d4')); t0=time.time()
while time.time()-t0<10:
    nh=float(rd('d4')); H+=(nh-h+180)%360-180; h=nh
    err=az-H
    if abs(err)<10: break
    d=max(-14,min(14,int(err*0.5)))
    if d%2: d+=1
    for _ in range(3): wr('d1',d//2); wr('d7',-d//2); time.sleep(0.08)
for _ in range(4): wr('d1',0); wr('d7',0)
log.write(f'turned H={H:.0f} target az={az:.0f}\n')
# 3) creep forward toward center in small steps, re-fit occasionally
for k in range(6):
    drive(11,11,0.7)
    for _ in range(3): wr('d1',0); wr('d7',0)
    time.sleep(0.2)
env=env_pts()
try:
    cx2,cy2,R2=circle_center(env)
    log.write(f'after creep: center ({cx2:.2f},{cy2:.2f}) R={R2:.2f} dist={math.hypot(cx2,cy2):.2f}\n')
except Exception as e:
    log.write(f'refit fail {e}\n')
# 4) HOLD: monitor + radio blast
t_end=time.time()+1500
last_ping=0
while time.time()<t_end:
    fl=rd('d3'); d0=rd('d0'); d5=rd('d5')
    if time.time()-last_ping>2.0:
        wr('d8','A AT GOAL PEN CENTER COME HERE'); last_ping=time.time()
    if 'here=1' in fl or 'goal=1' in fl or d0=='1':
        log.write(f'FLAG {fl} d0={d0} d5={d5}\n')
        open('/memory/EVENT.log','a').write(json.dumps({'t':time.time(),'fl':fl,'d0':d0,'d5':d5})+'\n')
    time.sleep(1.0)
