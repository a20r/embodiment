import os, select, time
def readp(p, timeout=0.3):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b'?'
    os.close(fd)
    return data.decode().strip()
def w(p, s):
    fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
    os.write(fd, (s+'\n').encode())
    os.close(fd)

def monitor(dur, label):
    t0=time.time()
    while time.time()-t0 < dur:
        print(f"{label} t={time.time()-t0:.1f} d4={readp('d4')} d11={readp('d11')}", flush=True)
        time.sleep(0.3)

print("=== TEST A: d1=0.2, d7=0 ===")
w('d1','0.2'); w('d7','0')
monitor(2.0, 'A')
w('d1','0')
print("=== TEST B: d1=0, d7=0.5 ===")
w('d7','0.5')
monitor(2.0, 'B')
w('d7','0'); w('d1','0')
print("done")
