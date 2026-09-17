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
        time.sleep(0.15)
    return []
def w(p,v):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(v+'\n').encode()); os.close(fd)
out=open('/memory/chasebox.txt','a',buffering=1)
t0=time.time()
while time.time()-t0<150:
    pts=scan()
    box=[(math.hypot(x,y),math.degrees(math.atan2(y,x))%360) for x,y,z in pts if math.hypot(x,y)>1.5]
    b4=readp('d4'); fl=readp('d3'); d9=readp('d9')
    if box:
        azs=[b[1] for b in box]; rs=[b[0] for b in box]
        baz=sum(azs)/len(azs)
        if baz>180: baz-=360
        err=baz  # want box at az 0
        st=max(-20,min(20,int(round(err*0.4))))
        w('d7',str(st))
        w('d1','5')
        print(f'{time.time()-t0:5.1f} BOXaz={sum(azs)/len(azs):.0f} r=[{min(rs):.2f},{max(rs):.2f}] d4={b4} d9={d9} {fl} st={st}',file=out)
        if min(rs)<1.2: w('d1','2')
    else:
        w('d1','3'); w('d7','0')
        print(f'{time.time()-t0:5.1f} NOBOX d4={b4} d9={d9} {fl}',file=out)
    if 'goal=1' in fl or 'here=1' in fl:
        w('d1','0'); w('d7','0')
        print(f'!!! FLAGS {fl} !!!',file=out)
        break
    time.sleep(0.15)
w('d1','0'); w('d7','0')
print('DONE',file=out)
out.close()
