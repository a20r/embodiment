import os, time
from driver import Driver, rd, lidar
d=Driver(); d.set(0,0)
t0=time.time(); i=0
while time.time()-t0<600:
    try:
        fd=os.open('/dev/robot/d8',os.O_WRONLY)
        os.write(fd,('A-TO-B: A HERE. HOLDING. COME. beep%d'%i).encode()+b'\n'); os.close(fd)
    except Exception: pass
    i+=1
    time.sleep(8)
d.close()
