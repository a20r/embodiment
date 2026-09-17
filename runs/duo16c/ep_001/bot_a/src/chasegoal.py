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
        time.sleep(0.12)
    return []
def w(p,v):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(v+'\n').encode()); os.close(fd)
out=open('/memory/chasegoal.txt','a',buffering=1)
t0=time.time(); n=0
while time.time()-t0<240:
    n+=1
    fl=readp('d3'); d4=readp('d4'); d9=readp('d9')
    if 'goal=1' in fl or 'here=1' in fl:
        w('d1','0'); w('d7','0')
        print(f'!!! FLAGS {fl} d4={d4} d9={d9} !!!',file=out); break
    try: err=((float(d4)+180)%360)-180
    except: err=0
    st=max(-20,min(20,int(round(err*1.5))))
    w('d7',str(st)); w('d1','6')
    if n%6==0:
        w('d8','R1 CHASING BEACON d9=%s'%d9)
        pts=scan()
        blobs=[(round(math.hypot(x,y),2),round(math.degrees(math.atan2(y,x))%360)) for x,y,z in pts if math.hypot(x,y)>1.5]
        print(f'{time.time()-t0:6.1f} d4={d4} d9={d9} {fl} blobs={blobs[:8]}',file=out)
    time.sleep(0.1)
w('d1','0'); w('d7','0')
print('CHASEDONE',file=out)
out.close()
