import os, select, time
def rd(p):
    fd = os.open('/dev/robot/'+p, os.O_RDONLY | os.O_NONBLOCK)
    out=[]
    t0=time.time()
    while time.time()-t0<0.3:
        r,_,_=select.select([fd],[],[],0.05)
        if r:
            try: out.append(os.read(fd,4096).decode().strip())
            except: pass
    os.close(fd)
    return out[-1] if out else '?'
def wr(p,s):
    fd=os.open('/dev/robot/'+p, os.O_WRONLY)  # may block briefly
    os.write(fd,(s+'\n').encode()); os.close(fd)
def sample(tag):
    v={p:rd(p) for p in ['d0','d2','d4','d5','d6','d9','d11']}
    print(tag, {k:(v[k][:40] if k=='d2' else v[k]) for k in v}, flush=True)
print("BASELINE 5s"); t0=time.time()
while time.time()-t0<5: sample("B")
print("CMD d1=0.5 d7=0.5 for 5s")
wr('d1','0.5'); wr('d7','0.5'); t0=time.time()
while time.time()-t0<5: sample("F")
print("STOP 0 0, watch 5s")
wr('d1','0'); wr('d7','0'); t0=time.time()
while time.time()-t0<5: sample("S")
