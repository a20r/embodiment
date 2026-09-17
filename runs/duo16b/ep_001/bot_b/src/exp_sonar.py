import sys; sys.path.insert(0,'/bot/src')
from lib import *
import os, time, select, statistics as st

stop(); time.sleep(0.5)
print("d11 static:", last_of('d11'), "h:", last_of('d4'), flush=True)
log=[]
write('d1',40); write('d7',40)
t0=time.time()
while time.time()-t0<4:
    v=last_of('d11',0.12)
    if v:
        log.append((time.time()-t0, float(v)))
        if float(v)<0.15: break
stop()
for t,v in log[:36]: print(f"{t:.2f} {v:.3f}", flush=True)
print("n=",len(log),"min=",min(x[1] for x in log), flush=True)
buf=''
fd=os.open('/dev/robot/d2', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time()
while time.time()-t0<3:
    r,_,_=select.select([fd],[],[],0.05)
    if r:
        try: buf+=os.read(fd,4096).decode()
        except BlockingIOError: pass
os.close(fd)
open('/memory/d2frame.txt','w').write(buf)
pts=[tuple(map(float,p.split(','))) for p in buf.split(';') if p and ',' in p]
c0=[p[0] for p in pts]; c1=[p[1] for p in pts]; c2=[p[2] for p in pts]
print("d2 n=",len(pts),"c0:",round(min(c0),3),round(max(c0),3),"c1:",round(min(c1),3),round(max(c1),3),"c2:",round(min(c2),3),round(max(c2),3))
