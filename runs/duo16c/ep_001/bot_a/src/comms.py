import os, select, time
def readp(p, timeout=2.0):
    fd = os.open(f'/dev/robot/{p}', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],timeout)
    data = os.read(fd, 65536) if r else b'<nodata>'
    os.close(fd)
    return data.decode().strip()
print("d10:", readp('d10'))
print("d3:", readp('d3'))
# try transmitting a hello on d8
try:
    fd = os.open('/dev/robot/d8', os.O_WRONLY)
    os.write(fd, b'HELLO\n')
    os.close(fd)
    print("d8 write ok")
    time.sleep(1.0)
    print("d10 after tx:", readp('d10'))
except Exception as e:
    print("d8 fail:", e)
