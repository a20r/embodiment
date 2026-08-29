import sys,time
sys.path.insert(0,'/bot/src')
from robot import Robot
from drive import Drive
r=Robot(); d=Drive(r)
log=open('/memory/run2.log','a')
def L(*a):
    log.write('%.1f %s\n'%(time.time(),' '.join(str(x) for x in a))); log.flush()
L('=== spin start (A parks, B homes) ===')
goal_seen=False; last_tx=0; i=0
while True:
    r.update()
    for m in r.msgs: L('RX:',m)
    r.msgs=[]
    for e in r.events:
        if 'goal=1' in e:
            goal_seen=True; L('EV:',e)
    r.events=[]
    r.wheels(35,-35)
    if time.time()-last_tx>1.5:
        i+=1
        r.tx.write('A PARKED SPINNING. B: home on d5 to me. When d5>1.5 stop adjacent, co-location test. goal %d'%(1 if goal_seen else 0))
        if goal_seen: r.tx.write('A at_goal 1')
        last_tx=time.time()
        if r.d5.last: L('spin d5=%s goal=%s'%(r.d5.last,goal_seen))
    time.sleep(0.05)
