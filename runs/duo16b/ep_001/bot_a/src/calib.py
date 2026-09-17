import os, select, time

D='/dev/robot/'
def read(p, timeout=0.25):
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
    except Exception as e: print("werr",p,e)

def enc(): 
    a=read('d6'); b=read('d9'); return float(a), float(b)

def lstats():
    s=read('d2'); pts=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try: pts.append(float(p.split(',')[0]))
        except: pass
    pos=[v for v in pts if v>0.02]
    if not pos: return None
    pos.sort()
    return dict(n=len(pos), min=pos[0], p25=pos[len(pos)//4], mean=sum(pos)/len(pos), max=pos[-1])

def drive(l,r,dur):
    t0=time.time()
    while time.time()-t0<dur:
        w('d1',l); w('d7',r); time.sleep(0.1)
    w('d1',0); w('d7',0)

r0,l0 = enc()
print("start enc", r0, l0, "lidar", lstats())
drive(60,60,1.0); time.sleep(0.3)
r1,l1 = enc()
print("after 1s fwd: enc", r1, l1, "d Enc: %.0f %.0f" % (r1-r0, l1-l1*0), "lidar", lstats())
drive(60,60,1.0); time.sleep(0.3)
r2,l2 = enc()
print("after 2s: enc", r2, l2, "dEnc: %.0f %.0f" % (r2-r1, l2-l1), "lidar", lstats())
drive(60,60,1.0); time.sleep(0.3)
r3,l3 = enc()
print("after 3s: enc", r3, l3, "dEnc: %.0f %.0f" % (r3-r2, l3-l2), "lidar", lstats())
