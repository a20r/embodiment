import os, select, time
def readp(p, timeout=0.5):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b'<nodata>'
    os.close(fd)
    return data.decode().strip()
for i in range(8):
    print(f"t={i*0.4:.1f}", "d0=",readp('d0'), "d4=",readp('d4'), "d5=",readp('d5'),
          "d6=",readp('d6'), "d9=",readp('d9'), "d11=",readp('d11'), "d3=",readp('d3'), flush=True)
    time.sleep(0.4)
