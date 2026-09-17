import os,select,math,time,collections
def readp(p,timeout=0.3):
    try:
        fd=os.open(f'/dev/robot/{p}',os.O_RDONLY)
        r,_,_=select.select([fd],[],[],timeout)
        d=os.read(fd,1<<22) if r else b''
        os.close(fd); return d.decode().strip()
    except Exception: return ''
def scan():
    for _ in range(4):
        d=readp('d2',1.5)
        if len(d)>10: return [tuple(map(float,p.split(','))) for p in d.split(';') if p]
        time.sleep(0.2)
    return []
LOG=open('/memory/survey.txt','a',buffering=1)
t0=time.time()
while time.time()-t0<95:
    d4=readp('d4'); d9=readp('d9'); d11=readp('d11'); d5=readp('d5'); d0=readp('d0'); d3=readp('d3')
    pts=scan()
    BIG=[]; H=[]
    for x,y,z in pts:
        rr=math.hypot(x,y); az=math.degrees(math.atan2(y,x))%360
        if rr>0.95: BIG.append((rr,az))
        elif 0.5<rr<=0.6: H.append(az)
    b=f'BIG:n={len(BIG)}'
    if BIG:
        rs=[a[0] for a in BIG]; azs=[a[1] for a in BIG]
        b+=f' r={sum(rs)/len(rs):.2f} az={sum(azs)/len(azs):.0f} rmin={min(rs):.2f} rmax={max(rs):.2f}'
    h=f'H:n={len(H)} az={sum(H)/len(H):.0f}' if H else 'H:n=0'
    LOG.write(f'{time.time()-t0:5.1f} d4={d4} d9={d9} d11={d11} d5={d5} d0={d0} {d3} {b} {h}\n')
    time.sleep(1.0)
LOG.write('SURVEY DONE\n')
