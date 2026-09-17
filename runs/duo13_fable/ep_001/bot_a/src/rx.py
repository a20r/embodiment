import sys,time
sys.path.insert(0,'/bot/src')
from rio import *
out=open('/tmp/rx.txt','a')
while True:
    s=readline(10,5.0)
    if s: out.write(f"{time.strftime('%H:%M:%S')} {s}\n"); out.flush()
    else: time.sleep(0.5)
