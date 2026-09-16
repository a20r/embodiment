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

def snap(label):
    print(f"{label}: d0={readp('d0')} d5={readp('d5')} d11={readp('d11')} d4={readp('d4')} d6={readp('d6')} d9={readp('d9')} d3={readp('d3')}", flush=True)

snap("start")
t0=time.time()
while time.time()-t0<3: w('d7','10'); time.sleep(0.05)
w('d7','0'); time.sleep(0.5)
snap("after rot1 (-26deg)")
t0=time.time()
while time.time()-t0<3: w('d7','10'); time.sleep(0.05)
w('d7','0'); time.sleep(0.5)
snap("after rot2 (-26deg)")
t0=time.time()
while time.time()-t0<3: w('d1','3'); time.sleep(0.05)
w('d1','0'); time.sleep(0.5)
snap("after fwd 3s")
