import os, time, select

def read_now(names, dur=1.2):
    fds={n:os.open('/dev/robot/'+n,os.O_RDONLY|os.O_NONBLOCK) for n in names}
    last={}
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select(list(fds.values()),[],[],0.05)
        for fd in r:
            for n,f in fds.items():
                if f==fd:
                    try: last[n]=os.read(f,4096).decode().strip()
                    except Exception: pass
    for f in fds.values(): os.close(f)
    return last

print("sample:", read_now(['d4','d6','d9','d3']))
