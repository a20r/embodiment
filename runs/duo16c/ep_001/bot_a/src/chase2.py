import os, select, time, math
def readp(p, timeout=0.12):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 65536) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def fl():
    t=readp('d3')
    try:
        parts=dict(kv.split('=') for kv in t.split() if '=' in kv)
        return f"g={parts.get('goal')} h={parts.get('here')}"
    except Exception: return "?"
t0=time.time(); n=0; last=None
while time.time()-t0<38:
    b=readp('d4'); d9=readp('d9')
    try: err=((float(b)+180)%360)-180
    except Exception: err=0
    w('d7', str(max(-22,min(22,int(round(err*2))))))
    w('d1','10')
    n+=1
    if n%6==0:
        try: rate = (int(d9)-last)/1.2 if last else 0
        except Exception: rate=0
        print(f"t={time.time()-t0:.0f} d4={b} {fl()} d9={d9} rate={rate:.0f}", flush=True)
        last=int(d9) if d9 else last
        try:
            fd=os.open('/dev/robot/d8',os.O_WRONLY)
            os.write(fd,f"R1 MAX SPEED d9={d9}\n".encode()); os.close(fd)
        except Exception: pass
    time.sleep(0.1)
w('d1','4')
print("END", fl())
