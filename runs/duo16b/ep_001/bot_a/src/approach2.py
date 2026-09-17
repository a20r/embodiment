import os,select,time,math
D='/dev/robot/'
def read(p,timeout=0.25):
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
def tx(m):
    try:
        fd=os.open(D+'d8',os.O_WRONLY|os.O_NONBLOCK); os.write(fd,m.encode()); os.close(fd)
    except Exception: pass
def fl(x,d=0.0):
    try: return float(x)
    except: return d
def objrange():
    pts=[]
    for p in read('d2').split(';'):
        p=p.strip()
        if not p: continue
        try:
            r,e,a=map(float,p.split(',')); pts.append((r,e,a))
        except: pass
    tall=[(r,a) for r,e,a in pts if e>0.4 and 0.05<r<2.5]
    if not tall: return None,None,0
    tall.sort()
    rs=[t[0] for t in tall]; azs=[t[1] for t in tall]
    return rs[len(rs)//2], azs[len(azs)//2], len(tall)
log=open('/memory/approach2.log','w',buffering=1)
log.write("=== APPROACH2: drive at tall object bearing 268, watch d11+d3\n")
fd10=os.open(D+'d10',os.O_RDONLY|os.O_NONBLOCK)
t0=time.time()
lasttx=0
while time.time()-t0<180:
    H=fl(read('d4'))
    s3=read('d3',0.15)
    d11=read('d11',0.15)
    # rx check
    r,_,_=select.select([fd10],[],[],0.02)
    if r:
        try: rx=os.read(fd10,4096).decode(errors='replace').strip()
        except: rx=''
        if rx: log.write(f"RX!!! {rx!r}\n")
    orng,oaz,n=objrange()
    if 'goal=1' in s3 or 'here=1' in s3:
        w('d1',0); w('d7',0)
        log.write(f"!!!!! FLAG {s3} d11={d11} orng={orng}\n")
        tx("GOALFOUND I AM AT GOAL\n")
        time.sleep(60)
        break
    # steer to object az
    if orng is None or n<10:
        # lost: rotate gently toward last known bearing 268
        err=((268-H+180)%360)-180
        w('d1',max(-25,min(25,err))); w('d7',-max(-25,min(25,err)))
        time.sleep(0.15); log.write(f"A2 search h={H:.0f}\n")
        continue
    wb=(H+oaz*57.2958)%360
    err=((wb-H+180)%360)-180
    spd=max(-15,min(15,err*2))
    if orng>0.12:
        v=22
        w('d1',v+spd); w('d7',v-spd)
        time.sleep(0.15)
    else:
        w('d1',0); w('d7',0)
        tx("ROBOT1 AT OBJECT d11=%s\n"%d11)
    if time.time()-lasttx>3:
        lasttx=time.time()
        tx("R1 STATUS d11=%s obj=%.2f\n"%(d11,orng if orng else -1))
    log.write(f"A2 {time.time()-t0:.1f} h={H:.0f} obj={orng} n={n} d11={d11} {s3}\n")
    if d5l:=read('d5',0.1):
        if d5l.strip()=='1': log.write("CONTACT d5=1\n")
w('d1',0); w('d7',0)
log.write("APPROACH2 end\n")
