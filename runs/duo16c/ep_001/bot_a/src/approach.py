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

# drive forward gently, log d4/d11/flags
t0=time.time()
while time.time()-t0<8:
    w('d1','1'); time.sleep(0.05)
    if int((time.time()-t0)*5)%2==0:
        print(f"t={time.time()-t0:.1f} d4={readp('d4')} d11={readp('d11')} d9={readp('d9')} d3={readp('d3')}", flush=True)
w('d1','0')
print("stopped.", readp('d3'), readp('d11'), readp('d4'))
