import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time, json
motors(0,0)
t0=time.time()
log=open("/bot/src/beacon.log","a")
while True:
    d5=read_float("d5"); d6=read_port("d6")
    write_port("d0", json.dumps(dict(who="A",msg="A STATIONARY - home on my signal. I stay put until d5>0.9",d5=round(d5,3))))
    log.write(f"{time.time():.0f} d5={d5:.3f} {d6}\n"); log.flush()
    if "here=1" in d6 or "goal=1" in d6:
        print("GOALFLAG",d6,flush=True)
    time.sleep(2)
