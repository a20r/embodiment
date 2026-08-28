import time, statistics
from agent import Agent
a=Agent(); b=a.b
a.lg('AROUND start')
b.radio_send('A info: my d0 blipped 1 a few times (I think=line-of-sight detector), my d7=contact flag works, d9 goal=0 always, d6=signal. No goal seen in ~150 cells. Trying to circle the wall to you now, I follow right-hand wall. Keep holding + report if you see my blob.')
t0=time.time(); last_tx=0
mode_done=False
while time.time()-t0<300 and not mode_done:
    s=b.sc()
    h=b.heading()
    if h is None: continue
    now=time.time()
    if now-last_tx>4:
        b.p[6].poll(); sg=b.p[6].last
        b.radio_send(f'A circling, sig={sg}')
        last_tx=now
    for m in b.radio_recv():
        a.lg(f'RX: {m}')
        if 'blob' in m.lower() and ('yes' in m.lower() or 'see you' in m.lower() or 'moved' in m.lower()):
            mode_done=True
    b.p[0].poll()
    if b.p[0].last=='1':
        a.lg('AROUND: d0 SIGHT!')
        b.radio_send('A: I have line-of-sight (d0=1)!')
    # right-hand wall follow
    eff=[x if x>0 else 0.05 for x in s]
    front=min(eff[0],eff[1]*1.4,eff[15]*1.4)
    r=eff[12]
    if front<0.28:
        # turn left in place
        b.wheels(-16,16); time.sleep(0.25); b.stop()
        continue
    w=0.0
    w+=max(min((r-0.25)*-70,30),-30)
    if eff[13]<0.6 and eff[11]<0.6:
        w+=max(min((eff[13]-eff[11])*35,20),-20)
    if r>1.0: w=-38
    if eff[4]<0.15: w-=18
    v=0.3 if front>0.6 else 0.15
    if b.stalled(): v=0.55
    b.drive(v,w)
    time.sleep(0.1)
b.stop()
a.lg(f'AROUND done mode_done={mode_done}')
if mode_done:
    b.radio_send('A: LEADING now. Follow my blob. On goal: park+GOALFOUND.')
    a.run()
