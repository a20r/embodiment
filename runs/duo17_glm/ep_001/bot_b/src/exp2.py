import os, select, time
RPORTS=['d0','d2','d3','d4','d5','d6','d9','d11']
fds={}; latest={}
for p in RPORTS:
    fds[os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)]=p
wf1=os.open('/dev/robot/d1', os.O_WRONLY|os.O_NONBLOCK)
wf7=os.open('/dev/robot/d7', os.O_WRONLY|os.O_NONBLOCK)
def phase(tag,dur,cmd=None):
    print(f"== {tag} ==",flush=True)
    t0=time.time()
    if cmd: os.write(wf1,(cmd[0]+'\n').encode()); os.write(wf7,(cmd[1]+'\n').encode())
    n=0
    while time.time()-t0<dur:
        r,_,_=select.select(list(fds),[],[],0.1)
        for fd in r:
            try: latest[fds[fd]]=os.read(fd,4096).decode().strip().replace('\n',';')
            except: pass
        if time.time()-t0 > n*1.0:
            n+=1
            print(tag, {k:latest.get(k,'')[:46] for k in RPORTS}, flush=True)
    if cmd: os.write(wf1,b'0\n'); os.write(wf7,b'0\n')
phase("BASE",5)
phase("FWD",6,('0.5','0.5'))
phase("POST",5)
