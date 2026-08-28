import time, math, sys, json
sys.path.insert(0,'/bot/src')
from bot import IO, clean

io = IO()
log = open('/memory/trail.log','a', buffering=1)
t0 = time.time()
x=y=0.0
prev=None
last_ping=0
last_t=time.time()
state='follow'
flags0={}
SPEED=5   # cmd
MPS=0.5   # est m/s at cmd 5
turn_until=0
mode_turn=0

def rays(l):
    f = min(l[0], l[1], l[15])
    r = min(l[12], l[11])
    fr = l[14]
    fr2 = l[13]
    left = l[4]
    return f,r,fr,fr2,left

while True:
    io.poll(0.05)
    l = io.lidar(); h = io.heading()
    if l is None or h is None: continue
    l = clean(l, prev); prev = l
    now = time.time()
    dt = now - last_t; last_t = now
    f,r,fr,fr2,left = rays(l)

    # dead reckoning uses last command speed
    # (we set below; approximate with SPEED cmd -> MPS when driving)

    turn = 0.0; fwd = 0.0
    if state=='follow':
        if f < 0.20:
            state='turnleft'
        else:
            fwd = SPEED
            # steer: keep right dist ~0.22
            err = r - 0.22
            turn = 60*err
            # diagonal anticipation: if front-right diag close, veer left
            if fr < 0.20: turn -= 40
            if fr > 0.7 and r > 0.5:
                # right opening: arc right
                turn = 50
            turn = max(-80,min(80,turn))
            if f < 0.35: fwd = 2
    elif state=='turnleft':
        turn = -70; fwd = 0
        if f > 0.35:
            state='follow'
    io.drive(turn, fwd)
    # dead reckoning
    v = MPS*(fwd/SPEED) if fwd else 0.0
    x += v*dt*math.cos(math.radians(h))
    y += v*dt*math.sin(math.radians(h))

    # comms
    if now-last_ping > 2:
        last_ping = now
        io.send('PING from=alpha x=%.2f y=%.2f'%(x,y))
        log.write('%.1f POS x=%.2f y=%.2f h=%.1f f=%.2f r=%.2f st=%s d0=%s d7=%s d6=%s d9=%s\n'%(
            now-t0,x,y,h,f,r,state,io.latest.get(0),io.latest.get(7),io.latest.get(6),io.latest.get(9)))
    for m in io.msgs:
        log.write('%.1f RX %s\n'%(now-t0,m))
        print('RX', m, flush=True)
    io.msgs=[]
    # flag changes
    for n in [0,7]:
        v0 = flags0.get(n); v1 = io.latest.get(n)
        if v1 is not None and v1!=v0:
            flags0[n]=v1
            log.write('%.1f FLAG d%d=%s\n'%(now-t0,n,v1))
    st = io.latest.get(9,'')
    if 'goal=1' in st:
        log.write('%.1f GOAL REACHED %s\n'%(now-t0,st))
        io.drive(0,0)
        io.send('GOAL x=%.2f y=%.2f'%(x,y))
