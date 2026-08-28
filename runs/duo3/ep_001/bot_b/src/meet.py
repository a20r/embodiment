import time, statistics
from agent import Agent
a=Agent(); b=a.b
h=b.heading(); a.facing=round(h/90)%4*90
a.lg('MEET start')
b.radio_send('A: OK holding-partner, I am homing on your signal now. Stay put.')
t0=time.time()
while time.time()-t0<240:
    s=a.sig(1.5) or 0
    a.lg(f'MEET sig={s:.3f}')
    if s>1.1:
        break
    a.home_step()
b.stop()
s=a.sig(1.5) or 0
a.lg(f'MEET done sig={s:.3f}')
for i in range(5):
    b.radio_send(f'A: adjacent now (my sig={s:.2f}). I will now explore SLOWLY, follow me. If either finds goal: park + GOALFOUND broadcasts.')
    time.sleep(1)
a.run()
