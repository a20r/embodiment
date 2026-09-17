import time
from scene import read_line, stream

def enc():
    return float(read_line('d6',0.5) or 0), float(read_line('d9',0.5) or 0)

def rate(dur=2.0):
    a=enc(); t0=time.time(); time.sleep(dur); b=enc()
    return (b[0]-a[0])/dur, (b[1]-a[1])/dur

print('baseline rate:', rate())
print('baseline rate:', rate())
