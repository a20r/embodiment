import sys,time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/lighthouse.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
R.stop()
L('LIGHTHOUSE start: still + TX flood. d11=%.3f tx=%s'%(R.fget('d11'),R.status()[-10:]))
t0=time.time()
last_rep=0
peak=0
uniq=set()
while time.time()-t0<240:
    R.tx('B-TO-A: GOAL FOUND. B AT GOAL. A: DRIVE TO B ON d11 RISING NOW. COME COME.')
    v=R.fget('d11')
    if v>peak:
        peak=v
        if peak>0.78: L('PEAK %.3f'%peak)
    m=R.rx()
    if m:
        key=m[:40]
        if key not in uniq:
            uniq.add(key)
            L('RX %s'%m[:110])
    st=R.status()
    if 'ok' in st and time.time()-last_rep>20:
        last_rep=time.time()
        L('t=%.0f peak=%.3f tx=ok (A hears me)'%(time.time()-t0,peak))
    time.sleep(0.18)
L('LIGHTHOUSE done peak=%.3f'%peak)
