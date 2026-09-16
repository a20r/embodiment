import os, select, time, math, subprocess
def readp(p, timeout=0.15):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    d = os.read(fd, 1<<22) if r else b''
    os.close(fd); return d.decode().strip()
def w(p,s):
    fd=os.open(f'/dev/robot/{p}',os.O_WRONLY); os.write(fd,(s+'\n').encode()); os.close(fd)
def objs():
    c=readp('d2',1.2); out=[]
    for p in c.split(';'):
        if not p: continue
        x,y,z=map(float,p.split(','))
        r=math.hypot(x,y)
        if r>0.55: out.append((r, round(math.degrees(math.atan2(y,x)))))
    out.sort()
    if not out: return []
    pr=out[0][0]
    return [(round(r,1),a) for r,a in out if r>pr+0.8][:4]
def fl():
    t=readp('d3')
    try:
        parts=dict(kv.split('=') for kv in t.split() if '=' in kv)
        return f"g={parts.get('goal')} h={parts.get('here')}"
    except Exception: return "?"
pids=[p for p in subprocess.run(['pgrep','-f','mower.py'],capture_output=True,text=True).stdout.split() if open(f'/proc/{p}/comm').read().strip()=='python3']
for p in pids: os.kill(int(p),19)
w('d1','0'); w('d7','0')
t0=time.time()
ntx=0
while time.time()-t0<230:
    el=time.time()-t0
    if el-ntx>2.0:
        w('d8','COME R1 COME. I AM STOPPED. RESPOND.'); ntx=el
    if abs(el%5)<0.12:
        print(f"t={el:.0f} d4={readp('d4')} d11={readp('d11')} {fl()} far={objs()}", flush=True)
    time.sleep(0.1)
for p in pids: os.kill(int(p),18)
print("lure done, mower resumed")
