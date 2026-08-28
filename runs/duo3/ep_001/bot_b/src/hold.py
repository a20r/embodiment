import time, statistics
from agent import Agent
a=Agent(); b=a.b
a.lg('HOLD start - asking partner to home on us')
t0=time.time()
while time.time()-t0<600:
    b.radio_send('A: I HOLD STILL now. You home on my signal. When adjacent, say ADJACENT; then I lead-explore and you follow.')
    vals=[]; t1=time.time()
    while time.time()-t1<3:
        b.p[6].poll(); vals+=[float(x) for x in b.p[6].queue]; b.p[6].queue.clear()
        b.p[0].poll()
        time.sleep(0.05)
    s=statistics.mean(vals) if vals else 0
    see=b.p[0].last=='1'
    msgs=b.radio_recv()
    for m in msgs: a.lg(f'RX: {m}')
    a.lg(f'HOLD sig={s:.3f} see={see}')
    if s>1.1 or see or any('ADJACENT' in m for m in msgs):
        a.lg('HOLD: partner adjacent!')
        break
b.radio_send('A: adjacent confirmed. I lead-explore now, follow me. On goal: park+GOALFOUND.')
a.run()
