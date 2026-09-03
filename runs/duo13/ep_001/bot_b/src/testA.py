import time, math
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
def avg_lid(n=4):
    acc=[[] for _ in range(16)]
    for _ in range(n):
        L=lid()
        if L:
            for i,v in enumerate(L):
                if v>0: acc[i].append(v)
        time.sleep(0.06)
    return [sum(a)/len(a) if a else None for a in acc]
for trial in range(2):
    L0=avg_lid(5)
    o0=(float(read('d6')),float(read('d9')))
    t0=time.time()
    while time.time()-t0<1.0:
        motor(30,30); time.sleep(0.02)
    motor(0,0); time.sleep(0.3)
    L1=avg_lid(5)
    o1=(float(read('d6')),float(read('d9')))
    ticks=((o1[0]-o0[0])+(o1[1]-o0[1]))/2.0
    # fit d_i - d_1i*? simple: closing c_i = L0-L1 where both valid
    xs=[];ys=[]
    for i in range(16):
        if L0[i] and L1[i]:
            c=L0[i]-L1[i]
            b=math.radians(22.5*i)
            xs.append((math.cos(b),math.sin(b),c))
    n=len(xs)
    A=[[sum(x[0]*x[0] for x in xs),sum(x[0]*x[1] for x in xs)],[sum(x[0]*x[1] for x in xs),sum(x[1]*x[1] for x in xs)]]
    Bv=[sum(x[0]*x[2] for x in xs),sum(x[1]*x[2] for x in xs)]
    det=A[0][0]*A[1][1]-A[0][1]*A[1][0]
    if abs(det)>1e-6:
        a=(A[1][1]*Bv[0]-A[0][1]*Bv[1])/det
        b=(A[0][0]*Bv[1]-A[1][0]*Bv[0])/det
        amp=math.hypot(a,b)
        phase=math.degrees(math.atan2(b,a))  # closing max at bearing=phase
        print(f"trial{trial}: ticks={ticks:.0f} closing amp={amp:.3f}m at rel bearing {phase%360:.1f} (beam {((phase%360)/22.5):.1f})")
        if ticks>50: print(f"   KM = {amp/(ticks/5.3):.4f} m/(unit*s)")
    print("   closings:",[f"{i}:{(L0[i]-L1[i]):+.2f}" for i in range(16) if L0[i] and L1[i]])
    print("   d11=",read('d11'))
