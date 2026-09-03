import os,select,time,json
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
def snap():
    pts=[]
    for p in read('d2').split(';'):
        p=p.strip()
        if not p: continue
        try: pts.append(tuple(map(float,p.split(','))))
        except: pass
    tall=[(r,e,a) for r,e,a in pts if e>0.4 and 0.05<r<3.0]
    deep=[(r,e,a) for r,e,a in pts if e<-0.9 and r>0.1]
    far=[(r,e,a) for r,e,a in pts if r>1.2 and 0.02<r]
    floor=[r for r,e,a in pts if 0.02<r<0.5 and -0.3<e<0.1]
    def med(v):
        v=sorted(v); return v[len(v)//2] if v else None
    return dict(n=len(pts),tall=len(tall),
        tall_r=med([t[0] for t in tall]),tall_emax=max([t[1] for t in tall],default=None),
        tall_az=(min(t[2] for t in tall),max(t[2] for t in tall)) if tall else None,
        deep=len(deep),deep_r=med([t[0] for t in deep]),deep_emin=min([t[1] for t in deep],default=None),
        far=len(far),far_r=med([t[0] for t in far]),far_emax=max([t[1] for t in far],default=None),
        floor_med=med(floor))
log=open('/memory/scan360.log','w',buffering=1)
log.write("=== SCAN360 rotate CCW, snapshot/0.5s\n")
w('d1',22); w('d7',-22)
t0=time.time(); snaps=[]
while time.time()-t0<30:
    H=fl(read('d4'))
    s=snap(); s['h']=H; s['t']=round(time.time()-t0,1)
    snaps.append(s)
    log.write(json.dumps(s)+"\n")
    time.sleep(0.5)
w('d1',0); w('d7',0)
json.dump(snaps,open('/memory/scan360.json','w'))
log.write("SCAN360 done\n")
