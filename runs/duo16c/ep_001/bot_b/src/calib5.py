import math, time, collections
from scene import read_line as rl, stream

def scan_pts():
    for _ in range(5):
        s = rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

def prof(pts, nb=72):
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.degrees(math.atan2(y,x))
        b=int((a+180)/(360/nb))%nb
        az[b].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items()}

def est_rotation(P0, P1):
    # find shift s (in bins) minimizing sum |P1[(b+s)%nb]-P0[b]|
    nb = len(P0)
    best=None; bs=0
    for s in range(-nb//2, nb//2):
        d = sum(abs(P1[(b+s)%nb]-P0[b]) for b in range(nb))
        if best is None or d<best: best=d; bs=s
    return bs*360/nb, best/nb

def raw(): return rl('d6',0.4), rl('d9',0.4), rl('d0',0.4), rl('d5',0.4), rl('d11',0.4)

print('raw before:', raw())
P0 = prof(scan_pts()); h0=rl('d4',0.5)

print('TEST A: (1,0) 4s')
stream('1','0',4.0,0.05); stream('0','0',0.4)
P1 = prof(scan_pts()); h1=rl('d4',0.5)
ang,res = est_rotation(P0,P1); print(f'  heading {h0}->{h1}  rot~{ang:+.0f}deg res={res:.3f}')
print('raw after A:', raw())

P0=P1; h0=h1
print('TEST B: (0,1) 4s')
stream('0','1',4.0,0.05); stream('0','0',0.4)
P1 = prof(scan_pts()); h1=rl('d4',0.5)
ang,res = est_rotation(P0,P1); print(f'  heading {h0}->{h1}  rot~{ang:+.0f}deg res={res:.3f}')
print('raw after B:', raw())

P0=P1; h0=h1
print('TEST C: (3,3) 10s LONG FWD')
stream('3','3',10.0,0.05); stream('0','0',0.4)
P1 = prof(scan_pts()); h1=rl('d4',0.5)
ang,res = est_rotation(P0,P1); print(f'  heading {h0}->{h1}  rot~{ang:+.0f}deg res={res:.3f}')
print('raw after C:', raw())
close = ' '.join(f'{b}:{P1[b]:.2f}' for b in sorted(P1) if P1[b]<0.5)
print('  close bins now:', close)
