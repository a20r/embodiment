import os, time, threading, statistics
NB=os.O_NONBLOCK
def wr(p,v):
    fd=os.open(f'/dev/robot/{p}', os.O_WRONLY|NB)
    os.write(fd, f'{v}\n'.encode()); os.close(fd)
def rd(p):
    import select
    fd=os.open(f'/dev/robot/{p}', os.O_RDONLY|NB)
    out=b''; t0=time.time()
    while time.time()-t0<0.3:
        r,_,_=select.select([fd],[],[],0.1)
        if r:
            d=os.read(fd,8192)
            if d: out=d
            # keep draining to latest
            while True:
                r2,_,_=select.select([fd],[],[],0.02)
                if r2:
                    d2=os.read(fd,8192)
                    if d2: out=d2
                    else: break
                else: break
            break
    os.close(fd)
    return out.decode().strip()
def sample(n=20,dt=0.05):
    rows=[]
    for _ in range(n):
        d5=rd('d5')
        rows.append([float(x) for x in d5.split(',') if x!='-1.000'] if d5 else [])
        time.sleep(dt)
    if not rows: return None
    k=min(len(r) for r in rows)
    return [statistics.mean(r[i] for r in rows) for i in range(k)]
base=sample()
print('baseline:', [f'{x:.3f}' for x in base])
stop=False
def pump(p,v):
    while not stop:
        try: wr(p,v)
        except: pass
        time.sleep(0.01)
for val in ['0.8','-0.8']:
    stop=False
    th=threading.Thread(target=pump,args=('d4',val)); th.start()
    time.sleep(2.0); stop=True; th.join()
    s=sample()
    print(f'd4={val}:', [f'{x:.3f}' for x in s] if s else None)
