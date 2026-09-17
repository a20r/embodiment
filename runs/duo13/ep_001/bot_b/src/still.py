import time, threading
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
def radio_tx(m):
    try:
        with open(D+'d8','w') as f: f.write(m+"\n")
    except Exception: pass
hits=[]
def listener():
    while True:
        try:
            with open(D+'d10') as f:
                s=f.read().strip()
                if s:
                    hits.append(s)
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    print("RX!!!",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
print("STILL experiment: 100s stationary",flush=True)
vals=[]
t0=time.time(); last_tx=0
while time.time()-t0<100:
    if time.time()-last_tx>2.0:
        last_tx=time.time()
        radio_tx(f"R1 STATIC BEACON t={time.time()-t0:.0f}")
    v=readl('d11')
    if v:
        vals.append(float(v))
    time.sleep(0.5)
print("d11 series:",[f"{v:.2f}" for v in vals],flush=True)
import statistics
half=len(vals)//2
print(f"first-half mean={statistics.mean(vals[:half]):.3f} second-half mean={statistics.mean(vals[half:]):.3f}",flush=True)
print("rx hits:",hits,flush=True)
