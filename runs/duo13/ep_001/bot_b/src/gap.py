import time
D='/dev/robot/'
def readl(p,tries=3):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read()
            lines=[x.strip() for x in s.split('\n') if x.strip()]
            if lines: return lines[-1]
        except Exception: pass
        time.sleep(0.02)
    return None
def fnum(p):
    s=readl(p)
    try: return float(s)
    except: return None
def lid():
    s=readl('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
def motor(l,r):
    with open(D+'d1','w') as f: f.write(f"{l:.1f}\n")
    with open(D+'d7','w') as f: f.write(f"{r:.1f}\n")
def angdiff(a,b): return (a-b+180)%360-180
def rot_to(tgt,spd=30,maxt=2.2):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<5: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.05)
def burst(meters,spd=42,maxsec=2.5,stopdist=0.34):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return
    need=meters/0.006*5.3
    h_tgt=fnum('d4'); t0=time.time()
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        B=lid()
        if B:
            Bv=[b if b>0 else 9.9 for b in B]
            if min(Bv[15],Bv[0],Bv[1])<stopdist: break
        h=fnum('d4')
        e=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,e*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
# aim at widest beams and drive
for attempt in range(4):
    d0=fnum('d11')
    B=lid()
    if not B: continue
    Bv=[b if b>0 else 9.9 for b in B]
    bi=max(range(16),key=lambda i:Bv[i])
    if Bv[bi]<0.9: 
        print("no opening",flush=True); break
    h=fnum('d4')
    tgt=(h+22.5*bi)%360
    print(f"attempt{attempt}: beam{bi}={Bv[bi]:.2f} rot {h:.0f}->{tgt:.0f} d11={d0}",flush=True)
    rot_to(tgt)
    burst(0.45,stopdist=0.32)
    d1=fnum('d11')
    print(f"  -> d11 {d0}->{d1}",flush=True)
print("gap escape done d11=",fnum('d11'),flush=True)
