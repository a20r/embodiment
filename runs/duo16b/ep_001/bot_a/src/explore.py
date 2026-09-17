import os, select, time, math, json
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d

def bands():
    s=read('d2'); vals=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02: vals.append((r,e,a))
        except: pass
    if not vals: return 0,0,0
    mx=max(v[0] for v in vals)
    horiz=[v[0] for v in vals if -0.1<=v[1]<=0.3]
    med_h = sorted(horiz)[len(horiz)//2] if horiz else 0
    return mx, med_h, len(vals)

class Nav:
    def __init__(self):
        self.r0=fl(read('d6')); self.l0=fl(read('d9'))
        self.x=0.0; self.y=0.0; self.h=fl(read('d4'))
    def poll(self):
        r=fl(read('d6'),self.r0); l=fl(read('d9'),self.l0)
        dr=r-self.r0; dl=l-self.l0; self.r0=r; self.l0=l
        h=fl(read('d4'),self.h); self.h=h
        fwd=(dr+dl)/2.0
        a=math.radians(h)
        self.x+=fwd*math.cos(a); self.y+=fwd*math.sin(a)
        return fwd

nav=Nav()
logf=open('/memory/trail.log','a',buffering=1)
logf.write(f"=== EXPLORE start {time.time():.0f} h={nav.h:.0f}\n")

def rotate_to(target, tol=5, maxt=6):
    t0=time.time()
    while time.time()-t0<maxt:
        err=((target-nav.h+180)%360)-180
        if abs(err)<tol: break
        spd=max(-55,min(55,err*2.5))
        w('d1',spd); w('d7',-spd); time.sleep(0.1); nav.poll()
    w('d1',0); w('d7',0); time.sleep(0.15)

def panscan(step=30):
    h0=nav.h; out={}
    for k in range(int(360/step)):
        mx,mh,n = bands()
        out[round(nav.h,0)] = mx
        rotate_to(nav.h+step, tol=4, maxt=3)
    rotate_to(h0, tol=4, maxt=6)
    return out

last_event_check=0
stall=0
while True:
    s3=read('d3'); d0=read('d0'); d5=read('d5'); d11=read('d11')
    if 'goal=1' in s3 or 'here=1' in s3 or d5=='1' or d0=='1':
        logf.write(f"EVENT {time.time():.0f} x={nav.x:.1f} y={nav.y:.1f} h={nav.h:.0f} {s3} d0={d0} d5={d5} d11={d11}\n")
    mx, medh, n = bands()
    r0=fl(read('d6')); l0=fl(read('d9'))
    # drive forward up to 2s, monitoring stall
    t0=time.time(); moved=0
    while time.time()-t0<2.0:
        w('d1',60); w('d7',60); time.sleep(0.25)
        moved += nav.poll()
        s3=read('d3')
        if 'goal=1' in s3 or 'here=1' in s3:
            logf.write(f"GOALFLAG {time.time():.0f} x={nav.x:.1f} y={nav.y:.1f} {s3}\n")
    w('d1',0); w('d7',0); time.sleep(0.15)
    logf.write(f"POS {time.time():.0f} x={nav.x:.2f} y={nav.y:.2f} h={nav.h:.0f} maxr={mx:.2f} medh={medh:.2f} d11={d11} d5={d5}\n")
    if moved < 30:   # stalled
        stall+=1
        logf.write(f"STALL {stall} moved={moved:.0f}\n")
        if stall>=2:
            # back up and pan scan
            t0=time.time()
            while time.time()-t0<1.0:
                w('d1',-50); w('d7',-50); time.sleep(0.15); nav.poll()
            w('d1',0); w('d7',0)
            op=panscan(30)
            logf.write("PAN "+json.dumps(op)+"\n")
            best=max(op.items(), key=lambda kv: kv[1])
            rotate_to(best[0]+ (0 if abs(((best[0]-nav.h+180)%360)-180)<90 else 180), tol=5)
            stall=0
        else:
            rotate_to(nav.h+40, tol=5)
    else:
        stall=0
        # every loop, slight steer toward larger maxr if very close obstacle
        if medh>0 and medh<0.08:
            rotate_to(nav.h+50, tol=5)
