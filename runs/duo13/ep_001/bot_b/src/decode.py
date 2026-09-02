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
def counters():
    return read('d5'), read('d6'), read('d9'), read('d11'), read('d4')
def run(l,r,secs,label):
    a=counters(); t0=time.time()
    while time.time()-t0<secs:
        motor(l,r); time.sleep(0.02)
    motor(0,0); time.sleep(0.2)
    b=counters(); el=time.time()-t0
    print(f"{label:10} d5:{a[0]}->{b[0]} d6:{a[1]}->{b[1]} d9:{a[2]}->{b[2]} d11:{a[3]}->{b[3]} h:{a[4]}->{b[4]}", flush=True)
run(0,0,1.0,"idle")
run(40,40,2.0,"both+")
run(-40,-40,2.0,"both-")
run(40,-40,1.5,"spinCW")
run(-40,40,1.5,"spinCCW")
run(40,0,1.5,"left only")
run(0,40,1.5,"right only")
run(0,0,1.0,"idle2")
