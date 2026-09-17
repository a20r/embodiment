import sys,time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/hold.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
R.stop()
L('HOLD at goal. d11=%.3f status=%s'%(R.fget('d11'),R.status()))
t0=time.time()
uniq=set()
peak=0
while True:
    R.tx('B-TO-A: B AT GOAL (here=1). A: COME TO CARRIER. WE FINISH TOGETHER. COME!')
    v=R.fget('d11')
    if v>peak:
        peak=v
        if peak>0.78: L('PEAK %.3f t=%.0f'%(peak,time.time()-t0))
    m=R.rx()
    if m and m[:40] not in uniq:
        uniq.add(m[:40]); L('RX %s'%m[:110])
    st=R.status()
    if 'goal=1' in st:
        L('*** GOAL=1: A AT GOAL! BOTH HERE! t=%.0f ***'%(time.time()-t0))
    if 'here=0' in st:
        L('!!! here=0 LOST GOAL ZONE - reacquire! %s'%st)
        # drive forward slowly to reacquire
        R.motors(18,18); time.sleep(1.0); R.stop()
    time.sleep(0.16)
