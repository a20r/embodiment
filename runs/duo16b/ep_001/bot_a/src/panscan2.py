import os, select, time, math, json
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

def bands():
    s=read('d2'); up=[]; horiz=[]; down=[]
    for p in s.split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(','))
            if r>0.02:
                if e>0.4: up.append((r,a))
                elif e>-0.1: horiz.append((r,a))
                else: down.append((r,a))
        except: pass
    um = max(up)[0] if up else 0
    uma = max(up)[1] if up else 0
    hm = max(horiz)[0] if horiz else 0
    hma = max(horiz)[1] if horiz else 0
    return um, uma, hm, hma

h0 = fl(read('d4'))
res=[]
cur=h0
for k in range(12):
    um,uma,hm,hma = bands()
    res.append((round(cur,1), round(um,3), round(uma,3), round(hm,3), round(hma,3)))
    # rotate 30 deg
    t0=time.time()
    while time.time()-t0<4:
        err=((cur+30-fl(read('d4'),cur)+180)%360)-180
        if abs(err)<4: break
        spd=max(-50,min(50,err*2.5))
        w('d1',spd); w('d7',-spd); time.sleep(0.1)
    w('d1',0); w('d7',0); time.sleep(0.2)
    cur = fl(read('d4'), cur)
for r in res: print("h=%.1f upMax=%.2f@az%.2f horMax=%.2f@az%.2f" % r)
