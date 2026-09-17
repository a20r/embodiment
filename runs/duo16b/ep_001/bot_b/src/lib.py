import os, time, select

def read_port(name, dur=0.25):
    fd=os.open('/dev/robot/'+name, os.O_RDONLY|os.O_NONBLOCK)
    acc=''; t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],0.02)
        if r:
            try:
                d=os.read(fd,4096).decode()
                if d: acc+=d
            except BlockingIOError: pass
    os.close(fd)
    return acc

def last_of(port, dur=0.25):
    lines=[x for x in read_port(port,dur).split('\n') if x.strip()]
    return lines[-1] if lines else None

def write(port, val):
    fd=os.open('/dev/robot/'+port, os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd,(str(val)+"\n").encode()); os.close(fd)

def stop():
    write('d1',0); write('d7',0)

def d4norm(a,b):
    return (b-a+540)%360-180
