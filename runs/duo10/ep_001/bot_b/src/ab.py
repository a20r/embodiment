from rob import *
import time
from nav import rot_to
def front(): return ranges()[2]
r=ranges(); h=heading()
best=max(range(16), key=lambda i: r[i])
rot_to((h+22.5*(best-2))%360)
print('front',front())
for cmd in [(7,7),(7.5,6.5),(8,6),(7,7)]:
    f0=front(); motors(*cmd); time.sleep(1.5); stop(); time.sleep(0.2); f1=front()
    print(cmd, round(f0-f1,2), 'front now', round(f1,2))
