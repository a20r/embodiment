import time
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n"); print(*a)
def stop(): wr("d4",0); wr("d5",0)
def rate(dur):
    n=0;k=0;t0=time.time()
    while time.time()-t0<dur:
        v=rd("d7")
        if v in("0","1"): n+=1; k+=(v=="1")
    return k,n
best=(0,0)
L("=== micro search ===")
for step in range(14):
    k,n=rate(2.5)
    L(f"step {step} rate {k}/{n}")
    if k>0:
        L("ON GOAL PATCH! staying")
        break
    # step backward 0.08u
    wr("d4",-3); wr("d5",-3); time.sleep(0.35); stop(); time.sleep(0.2)
stop()
