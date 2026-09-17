import os,select,time
D='/dev/robot/'
def read(p,timeout=0.15):
    fd=os.open(D+p,os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,9999999).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,v):
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,f"{v}\n".encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d
log=open('/memory/revtest.log','w',buffering=1)
log.write("=== REVTEST: reverse 6s, forward 6s, reverse 6s; watch d11\n")
phases=[(-25,'REV'),(0,'STOP1'),(25,'FWD'),(0,'STOP2'),(-25,'REV2'),(0,'END')]
for v,name in phases:
    t0=time.time()
    while time.time()-t0<6:
        w('d1',v); w('d7',v)
        time.sleep(0.12)
        d11=read('d11',0.05); d5=read('d5',0.05); d0=read('d0',0.05)
        log.write(f"{name} d11={d11} d5={d5} d0={d0}\n")
    w('d1',0); w('d7',0); time.sleep(0.5)
log.write("end\n")
