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
for trial in range(3):
    L0=lid(); o0=(read('d6'),read('d9')); h0=read('d4')
    print(f"trial{trial}: front beam0={L0[0]:.3f} beam15={L0[15]:.3f} beam1={L0[1]:.3f} o={o0} h={h0}",flush=True)
    t0=time.time(); stop=False
    while time.time()-t0<1.5 and not stop:
        motor(30,30); time.sleep(0.02)
        L=lid()
        if L and min(L[15]%99 or 9,L[0] if L[0]>0 else 9,L[1] if L[1]>0 else 9)<0.22: stop=True
    motor(0,0); time.sleep(0.3)
    L1=lid(); o1=(read('d6'),read('d9')); h1=read('d4')
    try:
        tr=(float(o1[0])-float(o0[0])+float(o1[1])-float(o0[1]))/2.0
        dd=L0[0]-L1[0]
        print(f"  -> beam0 {L0[0]:.3f}->{L1[0]:.3f} (d={dd:+.3f}) ticks={tr:+.0f} h {h0}->{h1}",flush=True)
        if tr>80 and dd>0.02: print(f"  KM={dd/(tr/5.3):.4f} m/(unit*s)",flush=True)
    except Exception as e: print("err",e,o0,o1,flush=True)
