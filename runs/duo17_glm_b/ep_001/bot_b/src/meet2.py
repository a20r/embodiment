import sys,time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/meet2.log','a')
def L(s):
    try: LOG.write('%.1f %s\n'%(time.time()%100000,s)); LOG.flush()
    except Exception: pass
R.stop()
L('MEET2 start. HOLDING. d11 loop begins.')
i=0
while True:
    i+=1
    R.tx('B HOLDING STILL. A: YOU SEE ME ON LIDAR? DRIVE TO ME ALONG THAT BEAM. OR REPLY BEAM<k> + YOUR HDG. KEEP TXING.')
    t0=time.time()
    while time.time()-t0<1.6:
        m=R.rx()
        if m:
            L('RX %s'%m[:130])
        time.sleep(0.1)
    v=R.fget('d11')
    if i%5==0: L('d11=%.3f hdg=%.0f'%(v,R.heading()))
    if v>0.85: L('*** d11 HIGH %.3f - A ADJACENT ***'%v)
