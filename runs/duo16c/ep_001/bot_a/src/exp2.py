import os, select, time, math
def readp(p, timeout=0.15):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def fl():
    try:
        parts=dict(kv.split('=') for kv in readp('d3').split() if '=' in kv)
        return f"g={parts.get('goal')} h={parts.get('here')}"
    except Exception: return "?"
w('d1','0'); w('d7','0'); time.sleep(0.5)
print("pre-turn d4=",readp('d4'),"d6=",readp('d6'),flush=True)
t0=time.time()
while time.time()-t0<6: w('d7','-20'); time.sleep(0.05)
w('d7','0'); time.sleep(0.5)
print("post-turn d4=",readp('d4'),"d6=",readp('d6'),flush=True)
t0=time.time()
while time.time()-t0<40:
    w('d1','3'); time.sleep(0.05)
    if int(time.time()-t0)%2==0:
        print(f"t={time.time()-t0:.0f} d4={readp('d4')} {fl()} d6={readp('d6')} d9={readp('d9')}", flush=True)
w('d1','0')
print("END", fl(), readp('d4'))
