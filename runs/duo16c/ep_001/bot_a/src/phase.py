import os, select, time
def readp(p, timeout=0.2):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b''
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode()); os.close(fd)

def run(label, dur, writer=None):
    t0=time.time(); samples=[]
    while time.time()-t0<dur:
        if writer: writer()
        samples.append((readp('d4'),readp('d5'),readp('d6'),readp('d9'),readp('d11')))
        time.sleep(0.1)
    print(label, "d4:", [s[0] for s in samples[::3]])
    print(label, "d5:", [s[1] for s in samples[::3]])
    print(label, "d6:", [s[2] for s in samples[::3]])
    print(label, "d9:", [s[3] for s in samples[::3]])
    print(label, "d11:", [s[4] for s in samples[::3]], flush=True)

run("P1 idle   ", 3.0)
run("P2 d1=0   ", 3.0, lambda: w('d1','0'))
run("P3 d7=0   ", 3.0, lambda: w('d7','0'))
run("P4 idle   ", 3.0)
run("P5 d1=1   ", 3.0, lambda: w('d1','1'))
run("P6 idle   ", 3.0)
