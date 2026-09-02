import time
D='/dev/robot/'
def read(p):
    try:
        with open(D+p) as f: s=f.read().strip()
        return s if s else None
    except Exception: return None
def motor(l,r):
    with open(D+'d1','w') as f: f.write(f"{l}\n")
    with open(D+'d7','w') as f: f.write(f"{r}\n")
def lidar():
    s=read('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
# 1) idle watch d11 for 12s
print("IDLE WATCH")
vals=[]
for i in range(24):
    v=read('d11'); vals.append(v); time.sleep(0.5)
print("d11 idle:", vals)
# 2) rotate in place 90 deg, watch d11
h0=read('d4'); v0=read('d11')
print("rot test: h0",h0,"d11",v0)
t0=time.time()
while time.time()-t0<2.2:
    motor(25,-25); time.sleep(0.02)
motor(0,0); time.sleep(0.3)
print("after rot: h",read('d4'),"d11",read('d11'))
# 3) translate: aim at widest gap, drive, measure closing
L=lidar()
bi=max(range(16), key=lambda i:(L[i] if L[i]>0 else 0))
print("widest beam",bi,"val",L[bi])
h0=float(read('d4')); tgt=(h0+22.5*bi)%360
t0=time.time()
while time.time()-t0<2.0:
    h=float(read('d4')); err=((tgt-h+180)%360-180)
    motor(30+max(-12,min(12,err)), 30-max(-12,min(12,err))); time.sleep(0.02)
motor(0,0); time.sleep(0.2)
L2=lidar()
d6s=[read('d6'),read('d9')]
print("after drive: L=",L2)
print("d6,d9 now:",d6s, "d11:",read('d11'),"h:",read('d4'))
