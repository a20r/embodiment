import sys, time
sys.path.insert(0,'/bot/src')
from rio import *
out=open('/tmp/log.txt','a')
while True:
    t=time.time()
    st=readline(3,1.0); s11=readline(11,1.0)
    out.write(f"{t:.1f} {st} sig={s11}\n"); out.flush()
    time.sleep(max(0,1.0-(time.time()-t)))
