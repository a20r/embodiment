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
def rot(tgt,maxt=2.5):
    t0=time.time()
    while time.time()-t0<maxt:
        H=fl(read('d4'))
        err=((tgt-H+180)%360)-180
        if abs(err)<5: break
        spd=max(-40,min(40,err*2))
        w('d1',spd); w('d7',-spd); time.sleep(0.08)
    w('d1',0); w('d7',0); time.sleep(0.4)
res=[]
for i in range(8):
    tgt=(i*45)%360
    rot(tgt)
    time.sleep(1.0)
    a=fl(read('d11'),-1)
    t0=time.time()
    while time.time()-t0<3.0:
        w('d1',30); w('d7',30); time.sleep(0.12)
    w('d1',0); w('d7',0)
    time.sleep(1.0)
    b=fl(read('d11'),-1)
    H=fl(read('d4'))
    res.append((tgt,H,a,b,b-a))
    print(f"probe {tgt}: d11 {a:.3f}->{b:.3f} d={b-a:+.3f}")
import json; json.dump(res,open('/memory/bearingprobe.json','w'))
