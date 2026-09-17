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
w('d1','0'); w('d7','0'); time.sleep(1)
def pose(): 
    return f"d4={readp('d4')} d6={readp('d6')} d9={readp('d9')} d11={readp('d11')}"
print("start:", pose(), flush=True)
# drive straight open loop 4s
t0=time.time()
while time.time()-t0<4: w('d1','3'); time.sleep(0.05)
w('d1','0'); time.sleep(1)
print("after straight:", pose(), flush=True)
# rotate in place CW (d7<0) 6s
t0=time.time()
while time.time()-t0<6: w('d7','-20'); time.sleep(0.05)
w('d7','0'); time.sleep(1)
print("after rot CW:", pose(), flush=True)
# drive straight again 4s (new heading)
t0=time.time()
while time.time()-t0<4: w('d1','3'); time.sleep(0.05)
w('d1','0'); time.sleep(1)
print("after straight2:", pose(), flush=True)
