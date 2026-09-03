import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
r=Robot()
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== manual: request B geometry ===')
t0=time.time(); last_tx=0
while time.time()-t0<40:
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs[:]=[]
    for e in r.events:
        if 'goal=1' in e: L('EV:',e)
    r.events[:]=[]
    if time.time()-last_tx>3:
        r.tx.write('B: REPORT NOW: your compass heading, clearances North East South West in meters, your d5. Repeat every 3s. Stay parked spinning.')
        last_tx=time.time()
    time.sleep(0.05)
r.update()
L('MY heading=%s'%r.h)
L('MY rays=%s'%r.lidar.last)
L('MY d5=%s'%r.d5.last)
