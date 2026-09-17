import time
from scene import read_line as rl, stream

def enc(): return float(rl('d6',0.5) or 0), float(rl('d9',0.5) or 0)
def rate(dur=2.0):
    a=enc(); time.sleep(dur); b=enc()
    return round((b[0]-a[0])/dur,2), round((b[1]-a[1])/dur,2)

for v in ['0.8','0.9','1.5','1.9','2','-1','-2']:
    stream(v, '0', 2.0)
    print(f'd1={v}:', rate())
stream('0','0',2.0)
print('stop:', rate())
