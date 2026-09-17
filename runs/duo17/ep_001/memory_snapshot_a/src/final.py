import sys,time; sys.path.insert(0,'/bot/src')
import explore2 as E
from explore2 import O,go,turn,log,c,m
E.radio.__defaults__  # noop
# patch radio message
def rad(msg=None):
    return E.radio('A: homing on you by RSSI, stay still. my d11=%.2f'%(E.d11() or -1))
E.main.__globals__['radio']=rad
try:
    E.main()   # returns on here=1 (writes ALERT) or plan-sent
finally:
    c.stop()
stt=m.rd('d3') or ''
log('final: main returned, status %s'%stt)
t0=time.time()
while time.time()-t0<240:
    stt=m.rd('d3') or ''
    m.wr('d8',"A: GO! I have here=1 now (%s). Step OUT 0.4 and back IN NOW so we arrive within 1 min. Then report goal=?"%stt[-25:],0.3)
    time.sleep(3); rx=m.rd('d10')
    if rx: log('RX <<< %r'%rx)
    log('final loop %s'%stt)
    if 'goal=1' in stt: log('!!!!!!!! GOAL=1 !!!!!!!!'); open('/memory/NOTES.md','a').write('- ep1 %s GOAL=1 ACHIEVED: %s\n'%(time.strftime('%H:%M'),stt)); break
