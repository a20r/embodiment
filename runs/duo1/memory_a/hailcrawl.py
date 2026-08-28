import rob, walker, time
def ad(x): return (x+180)%360-180
bng=walker.bng
def drain():
    out=''
    while True:
        s=rob.rd(5,tries=1)
        if not s: break
        out+=s
    return out
t0=time.time()
# attach to block: face beacon, drive till blocked
walker.turn(ad(0-bng()))
o0=rob.odo(); rob.motors(18,18)
while rob.odo()-o0<450:
    L=rob.lidar()
    if 0<L[0]<0.22: break
    time.sleep(0.05)
rob.motors(0,0)
print("attached F=%.2f b=%.0f"%(rob.lidar()[0], bng()), flush=True)
lasttx=0
while time.time()-t0<540:
    if rob.goal(): print("GOAL!!!", flush=True); rob.motors(0,0); break
    now=time.time()
    if now-lasttx>0.4:
        rob.wr(6,"hello"); lasttx=now
    r=drain()
    if r:
        print(f"CONTACT {r!r} b={bng():.0f} t={now-t0:.0f}", flush=True)
        rob.motors(0,0)
        for msg in ("hello","ping","who are you","help","open","goal","password"):
            rob.wr(6,msg); time.sleep(0.8)
            rr=drain()
            print(f"  {msg} -> {rr!r}", flush=True)
    # pressure crawl: keep beacon at reading 65 (velocity has inward component)
    b=bng()
    e=ad(b-65)
    c=max(-12,min(12,0.6*e))
    rob.motors(14+c,14-c)
    time.sleep(0.06)
rob.motors(0,0); print("end", flush=True)
