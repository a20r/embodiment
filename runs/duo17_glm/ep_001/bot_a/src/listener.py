import sys, time, os
sys.path.insert(0,"/bot/src")
from robust import rline
seen=set()
log=open("/memory/rx.log","a")
while True:
    v=rline("d10",1.0)
    if v:
        t=time.time()
        line=f"{t:.1f} {v}\n"
        log.write(line); log.flush()
    time.sleep(0.05)
