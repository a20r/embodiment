from grid3 import Nav
import time, statistics
b=Nav(); time.sleep(0.2)
b.stop()
b.radio_send('A: HOLDING EXACTLY STILL for 3 min. Watching d7 contact + d9. Go.')
t0=time.time(); log=open('/memory/still.log','a',buffering=1)
while time.time()-t0<185:
    b.p[7].poll(); b.p[0].poll(); b.p[9].poll(); b.p[6].poll()
    d7=b.p[7].last; d0=b.p[0].last; d9=b.p[9].last; s6=b.p[6].last
    for m in b.radio_recv(): log.write(f'{time.time()-t0:6.1f} RX {m}\n')
    if d7=='1' or d0=='1' or (d9 and 'goal=0' not in d9):
        log.write(f'{time.time()-t0:6.1f} EVENT d7={d7} d0={d0} d9={d9} sig={s6}\n')
        b.radio_send(f'A: EVENT d7={d7} d0={d0} sig={s6}')
    time.sleep(0.2)
log.write('STILL done\n')
b.radio_send('A: 3min hold done. status?')
