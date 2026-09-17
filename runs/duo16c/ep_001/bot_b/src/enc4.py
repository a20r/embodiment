import time
from scene import read_line as rl, stream

def enc(): return float(rl('d6',0.5) or 0), float(rl('d9',0.5) or 0)
def rate(dur=2.0):
    a=enc(); time.sleep(dur); b=enc()
    return round((b[0]-a[0])/dur,2), round((b[1]-a[1])/dur,2)

print('base:', rate())
stream('0.3','0',3.0)
print('after stream d1=0.3,d7=0 3s:', rate(), rate())
stream('0','0.3',3.0)
print('after stream d1=0,d7=0.3 3s:', rate(), rate())
stream('0','0',2.0)
print('stop:', rate())
