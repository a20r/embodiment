from scene import *
import time

base = descriptor(); time.sleep(0.5); base2 = descriptor()
print('noise diff (no cmd):', round(diff(base, base2),4))

fmts = ['1.0','1','f 1.0','v 1.0','move 1','go','m 1 1','1,1','1;1','throttle 1','set 1','v=1.0','spd 1']
for f in fmts:
    stream(f, f, 2.0)
    d = diff(base, descriptor())
    print(f'{f!r:14} diff={d:.4f}')
