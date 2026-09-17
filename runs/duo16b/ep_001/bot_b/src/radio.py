import sys; sys.path.insert(0,'/bot/src')
from lib import *
import os, time, select

# transmit
fd8=os.open('/dev/robot/d8', os.O_WRONLY|os.O_NONBLOCK)
fd10=os.open('/dev/robot/d10', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time(); got=[]
i=0
while time.time()-t0<20:
    if int((time.time()-t0)*2)%2==0 and i<40:
        try: os.write(fd8, f"PING{i}\n".encode())
        except Exception: pass
        i+=1
    r,_,_=select.select([fd10],[],[],0.2)
    if r:
        try:
            d=os.read(fd10,4096).decode().strip()
            if d: got.append(d); print("RX:", d, flush=True)
        except Exception: pass
print("sent", i, "received", len(got))
