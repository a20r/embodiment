import os, select, time
D='/dev/robot/'
fd=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
n=0; t0=time.time(); hits=[]
while time.time()-t0<15:
    r,_,_=select.select([fd],[],[],0.02)
    if r:
        d=os.read(fd,4096)
        if d.strip():
            hits.append(d)
            n+=1
print("tight loop 15s: hits:", n, hits[:5])
# also verify d8 write bytes
fd8=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
print("d8 write returned:", os.write(fd8, b"ARE YOU THERE?\n"))
os.close(fd8)
time.sleep(2)
fd=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
r,_,_=select.select([fd],[],[],1.0)
print("d10 after 2s:", os.read(fd,4096) if r else "nothing")
