import rob, walker, time
def ad(x): return (x+180)%360-180
MSGS=["hello","hi","hey","ping","pong","marco","polo","open","open sesame","unlock","key","password",
"help","goal","home","base","beacon","who","name","id","status","report","42","xyzzy","knock knock",
"come in","over","sos","mayday","rescue","guide","follow","lead","where is the goal","exit"]
def listen(d=1.2):
    buf=''; te=time.time()
    while time.time()-te<d:
        c=rob.rd(5,tries=1)
        if c: buf+=c+'|'; te=time.time()
        time.sleep(0.04)
    return buf
def fuzz_here(tag):
    for m in MSGS:
        rob.wr(6,m)
        r=listen()
        if r: print(f"[{tag}] {m!r} -> {r!r}", flush=True)
    print(f"[{tag}] fuzz done b={walker.bng():.0f}", flush=True)
# 1) at current spot
fuzz_here("here")
# 2) dive to block wall and fuzz
walker.turn(ad(0-walker.bng()))
o0=rob.odo(); rob.motors(18,18)
while rob.odo()-o0<450:
    L=rob.lidar()
    if 0<L[0]<0.22: break
    time.sleep(0.05)
rob.motors(0,0)
fuzz_here("blockwall")
# 3) crawl 1/4 around block, fuzz again, repeat x4
import sys
for k in range(4):
    t0=time.time()
    while time.time()-t0<25:
        b=walker.bng()
        e=ad(b-65)
        c=max(-12,min(12,0.6*e))
        rob.motors(14+c,14-c)
        time.sleep(0.06)
    rob.motors(0,0)
    fuzz_here("crawl%d"%k)
print("all done", flush=True)
