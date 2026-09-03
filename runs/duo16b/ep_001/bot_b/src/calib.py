import sys; sys.path.insert(0,'/bot/src')
from lib import *
import time

def state():
    h=last_of('d4'); e6=last_of('d6'); e9=last_of('d9')
    return float(h), int(e6), int(e9)

def trial(tag, cmd, dur=2.0):
    stop(); time.sleep(0.4)
    h0,e6_0,e9_0=state()
    for p,v in cmd: write(p,v)
    t0=time.time(); time.sleep(dur)
    stop()
    h1,e6_1,e9_1=state()
    dt=time.time()-t0-0.05
    dh=d4norm(h0,h1)
    print(f"{tag}: d4={dh:+7.1f} d6={e6_1-e6_0:+5d} d9={e9_1-e9_0:+5d}", flush=True)

trial("spd25",  [('d1',25),('d7',25)])
trial("spd50",  [('d1',50),('d7',50)])
trial("spd100", [('d1',100),('d7',100)])
trial("L100R-100", [('d1',100),('d7',-100)])
trial("L-100R100", [('d1',-100),('d7',100)])
trial("L50R-50", [('d1',50),('d7',-50)])
trial("L100R50", [('d1',100),('d7',50)])
print("heading now:", last_of('d4'))
