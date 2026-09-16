import os,select,math,time
def readp(p,timeout=0.3):
    fd=os.open(f'/dev/robot/{p}',os.O_RDONLY)
    r,_,_=select.select([fd],[],[],timeout)
    d=os.read(fd,1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def scan():
    for _ in range(6):
        d=readp('d2',1.5)
        if len(d)>10: return [tuple(map(float,p.split(','))) for p in d.split(';') if p]
        time.sleep(0.1)
    return []
def w(p,v):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(v+'\n').encode()); os.close(fd)
out=open('/memory/longdrive.txt','a',buffering=1)
def log(m): out.write(f'[{time.time():.0f}] {m}\n')
log('LONGDRIVE START')
t0=time.time(); lastlog=0; lastrx=0; charging=False
while True:
    fl=readp('d3'); d4=readp('d4'); d9=readp('d9'); b=readp('d11'); d0=readp('d0')
    if 'goal=1' in fl or 'here=1' in fl:
        w('d1','0'); w('d7','0')
        log(f'!!!FLAGS {fl} d9={d9} d4={d4}')
        w('d8','R1 AT GOAL d9=%s peerACK?'%d9)
        # stay put, watch peer approach, broadcast
        tend=time.time()+900
        while time.time()<tend:
            fl2=readp('d3')
            if 'goal=0' in fl2 and 'here=0' in fl2 and time.time()>tend-880: pass
            w('d8','R1 AT GOAL %s'%readp('d9'))
            pts=scan()
            blobs=[(round(math.hypot(x,y),2),round(math.degrees(math.atan2(y,x))%360)) for x,y,z in pts if math.hypot(x,y)>1.5]
            log(f'ATGOAL wait fl={fl2} blobs={blobs[:6]}')
            time.sleep(1.5)
        break
    try: err=((float(d4)+180)%360)-180
    except: err=0
    try: bat=float(b)
    except: bat=0.5
    if bat<0.12 and not charging:
        charging=True; w('d1','0'); log(f'BATTERY LOW {bat} -> charging stop')
    if charging:
        w('d1','0'); w('d7','0')
        if bat>0.30: charging=False; log(f'BATTERY {bat} -> resume drive')
    else:
        st=max(-20,min(20,int(round(err*1.5))))
        w('d7',str(st)); w('d1','6')
    if time.time()-lastlog>5:
        lastlog=time.time()
        log(f'd4={d4} d9={d9} d11={b} d0={d0} {fl} chg={charging}')
    if time.time()-lastrx>5:
        lastrx=time.time()
        w('d8','R1 ENROUTE d9=%s d4=%s'%(d9,d4))
    time.sleep(0.1)
log('LONGDRIVE EXIT')
out.close()
