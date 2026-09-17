import sys, time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/radio2.log','a')
def L(s):
    LOG.write('%.2f %s\n'%(time.time()%100000,s)); LOG.flush()
msgs=[
 'ROBOT-B HERE. I COPY YOU LOUD AND CLEAR.',
 'I AM COMING TO FIND YOU. KEEP TRANSMITTING.',
 'CAN YOU HEAR ME? REPLY WITH YOUR STATUS.',
]
i=0
t_end=time.time()+3600
while time.time()<t_end:
    m=R.rx()
    if m: L('RX '+m.replace('\n',' | '))
    ok=R.tx(msgs[i%len(msgs)])
    st=R.status()
    L('TX %d ok=%s %s'%(i,ok,st))
    i+=1
    time.sleep(0.25)
