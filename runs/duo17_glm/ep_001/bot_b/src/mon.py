import os, select, time, sys
RPORTS=['d0','d2','d3','d4','d5','d6','d9','d11']
fds={}; latest={}
for p in RPORTS:
    fds[os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)]=p
t0=time.time(); lastprint=0; f=open(sys.argv[1],'w',buffering=1)
dur=float(sys.argv[2]) if len(sys.argv)>2 else 600
while time.time()-t0<dur:
    r,_,_=select.select(list(fds),[],[],0.2)
    for fd in r:
        try: latest[fds[fd]]=os.read(fd,4096).decode().strip().replace('\n',';')
        except: pass
    if time.time()-lastprint>0.5:
        lastprint=time.time()
        f.write(f"{time.time()-t0:7.1f} "+" ".join(f"{k}={latest.get(k,'?')}" for k in RPORTS)+"\n")
