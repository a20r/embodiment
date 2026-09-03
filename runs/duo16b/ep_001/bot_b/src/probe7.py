import os, time, select

def read_now(names):
    fds={n:os.open('/dev/robot/'+n,os.O_RDONLY|os.O_NONBLOCK) for n in names}
    last={}
    t0=time.time()
    while time.time()-t0<0.6:
        r,_,_=select.select(list(fds.values()),[],[],0.05)
        for fd in r:
            for n,f in fds.items():
                if f==fd:
                    try: last[n]=os.read(f,4096).decode().strip()
                    except Exception: pass
    for f in fds.values(): os.close(f)
    return last

def w(port,val):
    fd=os.open('/dev/robot/'+port,os.O_WRONLY|os.O_NONBLOCK)
    os.write(fd,(str(val)+"\n").encode()); os.close(fd)

def trial(tag, cmd, dur=2.0):
    a=read_now(['d4','d6','d9'])
    t0=time.time()
    for p,v in cmd: w(p,v)
    time.sleep(dur)
    w('d1',0); w('d7',0)
    b=read_now(['d4','d6','d9'])
    try: d4=(float(b['d4'])-float(a['d4'])+540)%360-180
    except: d4=float('nan')
    try: d6=int(b['d6'])-int(a['d6']); d9=int(b['d9'])-int(a['d9'])
    except: d6=d9=-1
    print(f"{tag}: dt={time.time()-t0:.1f}s d4={d4:+.1f} d6={d6:+d} d9={d9:+d}", flush=True)

trial("d1=50", [('d1',50)])
trial("d7=50", [('d7',50)])
trial("d1=100", [('d1',100)])
trial("d7=100", [('d7',100)])
trial("both=100", [('d1',100),('d7',100)])
trial("both=50", [('d1',50),('d7',50)])
trial("d1=100,d7=-100", [('d1',100),('d7',-100)])
