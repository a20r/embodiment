import time, math, json

DEV='/dev/robot/'
def rd(port):
    for _ in range(50):
        with open(DEV+port) as f:
            s=f.readline().strip()
        if s: return s
        time.sleep(0.05)
    return ''
def wr(port,val):
    with open(DEV+port,'w') as f:
        f.write(str(val)+'\n')
def scan():
    while True:
        s=rd('d2')
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: return v
        except: pass
        time.sleep(0.05)
def heading(): return float(rd('d4'))
def sig():
    try: return float(rd('d11'))
    except: return -1
def status(): return rd('d3')
def tx(m): wr('d8',m)
def stop(): wr('d1',0); wr('d7',0)
def norm(a):
    while a>180: a-=360
    while a<-180: a+=360
    return a

def turn_to(H, tol=5, timeout=25):
    t0=time.time()
    while time.time()-t0<timeout:
        h=heading(); err=norm(H-h)
        if abs(err)<tol:
            stop(); time.sleep(0.2)
            h=heading()
            if abs(norm(H-h))<tol+2: return True
            continue
        # w positive decreases heading ~0.8w deg/s
        w=max(min(err*-1.2,25),-25)
        if 0<w<8: w=8
        if -8<w<0: w=-8
        wr('d1',0); wr('d7',round(w,1))
        time.sleep(0.15)
    stop(); return False

def fwd_step(H, maxtime=8, stopdist=0.32):
    """Drive forward holding heading H. Returns (b0_start,b0_end,reason). Uses lidar stuck detect."""
    s=scan(); b0s=s[0] if s[0]>0 else 2.7
    hist=[]
    t0=time.time(); reason='time'
    while time.time()-t0<maxtime:
        s=scan()
        f=s[0] if s[0]>0 else 2.7
        f1=s[1] if s[1]>0 else 2.7
        f15=s[15] if s[15]>0 else 2.7
        hist.append(f)
        if f<stopdist or f1<0.16 or f15<0.16:
            reason='blocked'; break
        h=heading(); err=norm(H-h)
        v=16+max(min(1.2*err,8),-8)
        wr('d1',round(v,1)); wr('d7',20)
        time.sleep(0.12)
        if len(hist)>25 and abs(hist[-1]-hist[-20])<0.04 and min(hist[-20:])<2.5:
            reason='stuck'; break
    stop(); time.sleep(0.2)
    s=scan(); b0e=s[0] if s[0]>0 else 2.7
    return b0s,b0e,reason

def unstick(H):
    # reverse a bit
    t0=time.time()
    while time.time()-t0<2.0:
        wr('d1',20); wr('d7',16)
        time.sleep(0.12)
    stop()
