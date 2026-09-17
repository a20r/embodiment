import sys,time; sys.path.insert(0,'/bot/src')
from robot import Robot; from drive import Drive
r=Robot(); d=Drive(r)
t0=time.time()
while r.h is None and time.time()-t0<3: r.update(); time.sleep(0.05)
for tgt in (95,185):
    h=d.turn_to(tgt)
    print('target',tgt,'h=',h,'rays=',[None if v is None else round(v,2) for v in [r.ray(i) for i in range(16)]],flush=True)
