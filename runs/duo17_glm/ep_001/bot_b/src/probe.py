import os, select, time, sys
PORTS = ['d0','d2','d3','d4','d5','d6','d9','d11']
fds = {}
for p in PORTS:
    fd = os.open('/dev/robot/'+p, os.O_RDONLY | os.O_NONBLOCK)
    fds[fd] = p
dur = float(sys.argv[1]) if len(sys.argv)>1 else 10.0
t0 = time.time()
while time.time() - t0 < dur:
    r,_,_ = select.select(list(fds),[],[],0.2)
    vals = {}
    for fd in r:
        try:
            data = os.read(fd, 4096).decode().strip()
        except Exception:
            data = ''
        if data:
            vals[fds[fd]] = data.replace('\n',';')
    if vals:
        print(f"{time.time()-t0:6.1f} " + " | ".join(f"{k}={v}" for k,v in sorted(vals.items())), flush=True)
