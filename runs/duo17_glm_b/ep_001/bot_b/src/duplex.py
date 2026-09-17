import sys, time
sys.path.insert(0,'/bot/src')
import robot2 as R
LOG=open('/memory/radio2.log','a')
def L(s):
    LOG.write('%.2f %s\n'%(time.time()%100000,s)); LOG.flush()

msgs=[
 'B HERE. I HEAR YOU. I WILL LISTEN AFTER THIS BURST.',
 'SEND NOW: YOUR POSITION, BATTERY, WHAT HELP YOU NEED.',
 'AFTER THIS BURST I LISTEN 5S. GO AHEAD.',
]
def burst(dur=3):
    t0=time.time(); i=0
    while time.time()-t0<dur:
        ok=R.tx(msgs[i%len(msgs)])
        i+=1
        time.sleep(0.2)
def listen(dur=5):
    t0=time.time()
    got=[]
    while time.time()-t0<dur:
        m=R.rx()
        if m: got.append(m); L('RX '+m.replace('\n',' | '))
        time.sleep(0.02)
    return got

for cyc in range(12):
    L('CYCLE %d talk'%cyc)
    burst()
    L('CYCLE %d listen'%cyc)
    g=listen()
    if g: L('CYCLE %d GOT %d msgs'%(cyc,len(g)))
L('duplex done')
