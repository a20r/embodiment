import time, statistics, threading
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
def fnum(p,tries=3):
    s=readl(p,tries)
    try: return float(s)
    except: return None
def tx(m):
    try:
        with open(D+'d8','w') as f: f.write(m+"\n")
    except Exception: pass
RX=[]
def listener():
    while True:
        try:
            with open(D+'d10') as f:
                s=f.read().strip()
                if s:
                    RX.append(s)
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    print("RX:",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
print("SIT: stationary beacon 150s",flush=True)
tx("B1 STOPPED. Come to me via your d11! Stop at 0.25 and say B2 HERE!")
t0=time.time()
vals=[]
while time.time()-t0<150:
    v=fnum('d11')
    if v: vals.append((time.time()-t0,v))
    while RX:
        m=RX.pop(0)
        if 'HERE' in m or 'GOAL' in m.upper():
            print("B2 SAYS:",m,flush=True)
            with open('/memory/b2_here.flag','w') as g: g.write(m)
    time.sleep(0.5)
tx("B1 still stopped! keep coming! d11="+str(vals[-1][1] if vals else -1))
with open('/memory/sit.log','w') as g:
    for t,v in vals: g.write(f"{t:.0f},{v}\n")
print("sit done, d11 now",vals[-1][1] if vals else None,flush=True)
