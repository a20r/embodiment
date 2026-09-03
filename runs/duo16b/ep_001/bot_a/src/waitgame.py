import os, select, time, math
D='/dev/robot/'
def read(p, timeout=0.3):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,2000000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d

def sample_d11(n=4):
    vals=[]
    for i in range(n):
        v=fl(read('d11'),9.9)
        if v<9: vals.append(v)
        time.sleep(0.05)
    vals.sort()
    return vals[0] if vals else 9.9

logf=open('/memory/trail.log','a',buffering=1)
logf.write("=== WAITGAME: stop, ping, listen; if d11<0.32 burst TX\n")
# stop motors
w('d1',0); w('d7',0)
fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
t0=time.time(); burst=False
lastping=0
while time.time()-t0<420:
    now=time.time()
    d11=sample_d11()
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        logf.write(f"!!! FLAG {s3} d11={d11}\n")
    if now-lastping>2.0:
        lastping=now
        try:
            fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd, b"PING ROBOT1 STANDING HERE\n"); os.close(fd)
        except Exception: pass
    if d11<0.32 and not burst:
        burst=True
        logf.write(f"WAIT: companion close d11={d11:.3f}, switching to burst TX\n")
    r,_,_=select.select([fd10],[],[],0.05)
    if r:
        try: d=os.read(fd10,4096)
        except Exception: d=b''
        if d.strip(): logf.write(f"RX!!! {d!r}\n")
    if burst and int((now*2)%2)==0:
        try:
            fd=os.open(D+'d8', os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd, b"ROBOT1 HERE. ACK IF ALIVE\n"); os.close(fd)
        except Exception: pass
    if int(now)%10==0:
        logf.write(f"WAIT {now:.0f} d11={d11:.3f} burst={burst} {s3}\n")
        time.sleep(0.5)
logf.write("WAITGAME end\n")
