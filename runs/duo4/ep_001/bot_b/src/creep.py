import time
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n"); print(*a)
def stop(): wr("d4",0); wr("d5",0)
def d7():
    try: return rd("d7")=="1"
    except: return False
def probe(cl,cr,maxt):
    # move while polling d7 fast; stop on hit
    wr("d4",cl); wr("d5",cr)
    t0=time.time()
    while time.time()-t0<maxt:
        if d7():
            stop(); L(f"HIT d7 during move ({cl},{cr}) after {time.time()-t0:.2f}s")
            return True
        if rd("d0")=="1":
            stop(); L("bump"); return False
    stop()
    return False
stop()
L(f"=== creep start {time.time()} d7now={d7()}")
# 1) reverse slowly up to ~1.5u
if not probe(-4,-4,4.5):
    # 2) forward again a bit further
    if not probe(4,4,6.0):
        probe(-4,-4,3.0)
L(f"creep phase1 done d7={d7()}")
# settle check
for i in range(10):
    L(f"settle {i} d7={d7()}")
    time.sleep(0.3)
