import os,select,time,json,math
D='/dev/robot/'
def read(p,timeout=0.2):
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
def rot(tgt,maxt=3):
    t0=time.time()
    while time.time()-t0<maxt:
        H=fl(read('d4'))
        err=((tgt-H+180)%360)-180
        if abs(err)<4: break
        spd=max(-10,min(10,err*1.5))
        w('d1',spd); w('d7',-spd); time.sleep(0.1)
    w('d1',0); w('d7',0); time.sleep(1.2)
    return fl(read('d4'))
def snap():
    pts=[]
    for p in read('d2',0.3).split(';'):
        p=p.strip()
        if not p: continue
        try: pts.append(tuple(map(float,p.split(','))))
        except: pass
    return pts
H0=fl(read('d4'))
out=[]
for i in range(24):
    tgt=(H0+i*15)%360
    H=rot(tgt)
    pts=snap()
    tall=[(r,e,a) for r,e,a in pts if e>0.35 and 0.05<r<3.0]
    deep=[(r,e,a) for r,e,a in pts if e<-0.9 and r>0.05]
    far=[(r,e,a) for r,e,a in pts if r>0.95 and 0.02<r]
    mid=[(r,e,a) for r,e,a in pts if 0.1<=r<0.95 and -0.35<=e<=0.35]
    out.append(dict(h=H,tall=[(round(r,2),round(e,2),round(a,2)) for r,e,a in tall],
                    deep=[(round(r,2),round(e,2),round(a,2)) for r,e,a in deep],
                    far=[(round(r,2),round(e,2),round(a,2)) for r,e,a in far],
                    mid=[(round(r,2),round(e,2),round(a,2)) for r,e,a in mid]))
    print(f"{i} h={H:.0f} tall={len(tall)} deep={len(deep)} far={len(far)} mid={len(mid)}")
    if 'goal=1' in read('d3',0.1): print("FLAG GOAL")
json.dump(out,open('/memory/stepscan2.json','w'))
print("done")
