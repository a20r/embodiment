import time
D='/dev/robot/'
def read(p,tries=4):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.02)
    return None
def motor(l,r):
    with open(D+'d1','w') as f: f.write(f"{l}\n")
    with open(D+'d7','w') as f: f.write(f"{r}\n")
def lid():
    s=read('d2')
    return [float(x) for x in s.split(',')] if s else None
def odom():
    a,b=read('d6'),read('d9')
    try: return (float(a),float(b))
    except: return (None,None)
# 1) reverse ~0.5m worth of ticks
o0=odom()
t0=time.time()
while time.time()-t0<2.5:
    o=odom()
    if o0[0] is not None and o[0] is not None and (o[0]-o0[0])<-220: break
    motor(-30,-30); time.sleep(0.02)
motor(0,0); time.sleep(0.2)
print("reversed to o=",odom(),flush=True)
L=lid()
print("lidar:",[f"{v:.2f}" for v in L],flush=True)
# 2) rotate toward widest beam
bi=max(range(16),key=lambda i:(L[i] if L[i]>0 else 0))
h=float(read('d4'))
tgt=(h+22.5*bi)%360
def angdiff(a,b): return (a-b+180)%360-180
t0=time.time()
while time.time()-t0<4:
    h=float(read('d4'))
    err=angdiff(tgt,h)
    if abs(err)<6: break
    motor(22 if err>0 else -22, -22 if err>0 else 22); time.sleep(0.02)
motor(0,0)
print("rotated to h=",read('d4'),"target was",tgt,flush=True)
# 3) drive forward 1.2s, see which beam closes most -> TRUE FORWARD
L0=lid(); o0=odom()
t0=time.time()
while time.time()-t0<1.2:
    motor(28,28); time.sleep(0.02)
motor(0,0); time.sleep(0.2)
L1=lid(); o1=odom()
d=[(L0[i]-L1[i],i) for i in range(16) if L0[i]>0 and L1[i]>0]
d.sort(reverse=True)
print("ticks:",(o1[0]-o0[0]) if None not in (o1[0],o0[0]) else '?')
print("beam deltas (closed most first):",[(i,f"{dd:+.2f}") for dd,i in d[:6]],flush=True)
print("lidar now:",[f"{v:.2f}" for v in L1],flush=True)
print("d11:",read('d11'),"d5:",read('d5'),flush=True)
