import sys, time
sys.path.insert(0,'/bot/src')
from rob import *

def lastline(name, timeout=0.3):
    d = read_port(name, timeout).decode(errors='replace')
    lines=[l for l in d.strip().split('\n') if l.strip()]
    return lines[-1] if lines else ''

def drain(name):
    while True:
        d = read_port(name, 0.05)
        if not d: break

def drive(l, r, secs):
    write_port('d1', str(l)); write_port('d7', str(r))
    time.sleep(secs)
    write_port('d1','0'); write_port('d7','0')

# baseline cloud
p0 = scan(1.2)
back0 = [p for p in p0 if p[0] < -0.2]
bx0 = min([abs(p[0]) for p in back0]) if back0 else None
o9_0 = float(lastline('d9')); o6_0 = float(lastline('d6')); d11_0 = lastline('d11')
print('back xmin=%.3f d9=%.0f d6=%.0f d11=%s' % (bx0 if bx0 else -1, o9_0, o6_0, d11_0))
drive(-6,-6, 2.0)
time.sleep(0.4)
p1 = scan(1.2)
back1 = [p for p in p1 if p[0] < -0.2]
bx1 = min([abs(p[0]) for p in back1]) if back1 else None
o9_1 = float(lastline('d9')); o6_1 = float(lastline('d6')); d11_1 = lastline('d11')
print('after back 2s: back xmin=%.3f d9=%.0f d6=%.0f d11=%s' % (bx1 if bx1 else -1, o9_1, o6_1, d11_1))
print('d9 delta=%.0f d6 delta=%.0f' % (o9_1-o9_0, o6_1-o6_0))
print('moved back by: %.3f (lidar)' % ((bx1-bx0) if (bx0 and bx1) else -1))
time.sleep(0.5)
# drive toward heading 180 (behind at start? we're at ~325 now)
d04 = lastline('d4')
print('heading now:', d04)
drive(-6,-6, 2.5)
time.sleep(0.4)
print('d11 after more reverse:', lastline('d11'), 'd0=', lastline('d0'), 'd5=', lastline('d5'))
drive(-6,-6, 2.5)
time.sleep(0.4)
print('d11 after even more reverse:', lastline('d11'), 'd0=', lastline('d0'), 'd5=', lastline('d5'))
st = status(); print(st)
