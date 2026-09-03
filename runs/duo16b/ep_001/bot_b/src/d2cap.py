import os, time, select
buf=''
fd=os.open('/dev/robot/d2', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time()
while time.time()-t0<2.5:
    r,_,_=select.select([fd],[],[],0.05)
    if r:
        try: buf+=os.read(fd,4096).decode()
        except BlockingIOError: pass
os.close(fd)
open('/memory/d2static.txt','w').write(buf)
trip=[p for ln in buf.split('\n') for p in ln.split(';') if p and ',' in p]
print("total triplets:", len(trip))
for p in trip[:45]: print(p)
