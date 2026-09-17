import time
from scene import read_line, stream

def enc(): return float(read_line('d6',0.5) or 0), float(read_line('d9',0.5) or 0)
def rate(dur=2.0):
    a=enc(); time.sleep(dur); b=enc()
    return round((b[0]-a[0])/dur,2), round((b[1]-a[1])/dur,2)

# stop everything
stream('0','0',1.0)
print('after stop:', rate(), rate())
# single write of 0.3 to d1
from scene import read_line as rl
import os
fd=os.open('/dev/robot/d1', os.O_WRONLY); os.write(fd, b'0.3\n'); os.close(fd)
time.sleep(2)
print('after single d1=0.3:', rate(), 'd4:', rl('d4',0.5))
# single write of -0.3 to d7
fd=os.open('/dev/robot/d7', os.O_WRONLY); os.write(fd, b'-0.3\n'); os.close(fd)
time.sleep(2)
print('after single d7=-0.3:', rate(), 'd4:', rl('d4',0.5))
stream('0','0',1.0)
print('final stop:', rate())
