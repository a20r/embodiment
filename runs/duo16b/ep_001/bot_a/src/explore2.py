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

nav_r0=fl(read('d6')); nav_l0=fl(read('d9'))
nav_x=0.0; nav_y=0.0; nav_h=fl(read('d4'))
def poll():
    global nav_r0,nav_l0,nav_x,nav_y,nav_h
    r=fl(read('d6'),nav_r0); l=fl(read('d9'),nav_l0)
    dr=r-nav_r0; dl=l-nav_l0; nav_r0=r; nav_l0=l
    h=fl(read('d4'),nav_h); nav_h=h
    fwd=(dr+dl)/2.0
    a=math.radians(h)
    nav_x+=fwd*math.cos(a); nav_y+=fwd*math.sin(a)
    return fwd

def broadcast(msg):
    try:
        fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
        os.write(fd, msg.encode()); os.close(fd)
    except Exception: pass

logf=open('/memory/trail.log','a',buffering=1)
logf.write(f"=== EXPLORE2 spiral start {time.time():.0f} h={nav_h:.0f} x0reset\n")
broadcast("EXPLORE2 START\n")

SPIRAL_START=time.time()
TURN0 = 0.012   # deg of heading change per tick of travel (initial)
MIN_TURN = 0.0008
last_log=0.0
stall=0
try:
  while True:
    s3=read('d3'); d0=read('d0'); d5=read('d5')
    if 'goal=1' in s3:
        logf.write(f"!!! GOAL {time.time():.0f} x={nav_x:.1f} y={nav_y:.1f} h={nav_h:.0f} {s3}\n")
        w('d1',0); w('d7',0)
        while True:
            broadcast(f"GOALFOUND x={nav_x:.0f} y={nav_y:.0f}\n")
            time.sleep(2)
    if 'here=1' in s3 or d5=='1' or d0=='1':
        logf.write(f"EVENT {time.time():.0f} x={nav_x:.1f} y={nav_y:.1f} h={nav_h:.0f} {s3} d0={d0} d5={d5}\n")
    el = time.time()-SPIRAL_START
    turn = max(MIN_TURN, TURN0 * math.exp(-el/900.0))
    moved=0; t0=time.time(); k=0
    while time.time()-t0<1.0:
        # spiral: left faster than right by factor => turns left (heading+)
        w('d1', 100); w('d7', max(0, 100 - turn*3000))
        moved+=poll()
        time.sleep(0.15)
    w('d1',0); w('d7',0)
    if time.time()-last_log>5:
        last_log=time.time()
        logf.write(f"POS {time.time():.0f} x={nav_x:.0f} y={nav_y:.0f} h={nav_h:.0f} d0={d0} d5={d5} d11={read('d11')}\n")
    if moved<25:
        stall+=1
        logf.write(f"STALL {stall} moved={moved:.0f} at x={nav_x:.0f} y={nav_y:.0f}\n")
        t0=time.time()
        while time.time()-t0<0.8:
            w('d1',-60); w('d7',-60); time.sleep(0.15); poll()
        w('d1',0); w('d7',0)
        tgt=nav_h+90+30*stall
        t0=time.time()
        while time.time()-t0<4:
            err=((tgt-nav_h+180)%360)-180
            if abs(err)<5: break
            spd=max(-55,min(55,err*2.5))
            w('d1',spd); w('d7',-spd); time.sleep(0.1); poll()
        w('d1',0); w('d7',0)
        if stall>=6: stall=0
    else:
        stall=0
except Exception as e:
    logf.write(f"EXPLORE2 CRASH {e}\n")
    raise
