import os, time, select

def sample(names, dur):
    fds={n:os.open('/dev/robot/'+n, os.O_RDONLY|os.O_NONBLOCK) for n in names}
    data={n:[] for n in names}
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select(list(fds.values()),[],[],0.05)
        for fd in r:
            for n,f in fds.items():
                if f==fd:
                    try:
                        d=os.read(f,4096).decode().strip()
                        if d: data[n].append(d)
                    except Exception: pass
    for n,f in fds.items(): os.close(f)
    return data

def w(port, val):
    fd=os.open('/dev/robot/'+port, os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd, (str(val)+"\n").encode()); os.close(fd)

d=sample(['d4'],2)
print("baseline d4:", d['d4'][:2], d['d4'][-2:])
w('d1', 1.0)
d=sample(['d4'],3)
print("d1=1.0 d4:", d['d4'][:2], d['d4'][-2:], "n=",len(d['d4']))
w('d1', 0)
d=sample(['d4'],2)
print("d1=0 d4:", d['d4'][:2], d['d4'][-2:])
w('d7', 1.0)
d=sample(['d4'],3)
print("d7=1.0 d4:", d['d4'][:2], d['d4'][-2:], "n=",len(d['d4']))
w('d7', 0)
d=sample(['d4'],1)
print("d7=0 d4:", d['d4'][:2], d['d4'][-2:])
