import sys, time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/radio2.log','a')
def L(s):
    LOG.write('%.2f %s\n'%(time.time()%100000,s)); LOG.flush()
L('LISTEN-only start')
t_end=time.time()+25
while time.time()<t_end:
    m=R.rx()
    if m: L('RX '+m.replace('\n',' | '))
    time.sleep(0.02)
L('LISTEN done')
