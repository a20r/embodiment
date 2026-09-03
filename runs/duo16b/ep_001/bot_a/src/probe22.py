import os, select, time
D='/dev/robot/'
def read(p, timeout=0.2):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,200000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    if isinstance(msg,(int,float)): msg=f"{msg}\n"
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass

fd10=os.open(D+'d10', os.O_RDONLY|os.O_NONBLOCK)
w('d1',80); w('d7',80)
t0=time.time()
while time.time()-t0<12:
    d5=read('d5')
    r,_,_=select.select([fd10],[],[],0.05)
    rx=''
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
    if d5=='1' or rx:
        print(f"t={time.time()-t0:.1f} d5={d5} RX={rx!r}")
w('d1',0); w('d7',0)
print("done; d5 now:", read('d5'))
