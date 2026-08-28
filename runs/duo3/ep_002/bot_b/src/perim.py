import time, math, sys
from grid3 import Nav
from ctl import angdiff

b=Nav()
log=open('/memory/perim.log','a',buffering=1)
t0=time.time()
def lg(m): log.write(f'{time.time()-t0:7.1f} {m}\n')

# dead-reckoning pose
X,Y=0.0,0.0
last=time.time()
lastv=0.0
def upd(h):
    global X,Y,last
    now=time.time(); dt=now-last; last=now
    X+=lastv*math.cos(math.radians(h))*dt
    Y+=lastv*math.sin(math.radians(h))*dt

def drive(v,w,h):
    global lastv
    upd(h)
    b.drive(v,w); lastv=v

def checks():
    b.p[9].poll(); b.p[6].poll()
    if b.p[9].last and 'goal=0' not in b.p[9].last:
        lg(f'GOALFLAG {b.p[9].last} pose=({X:.2f},{Y:.2f})')
        print('GOAL!', flush=True)
    for m in b.radio_recv(): lg(f'RX {m}')

mode=sys.argv[1] if len(sys.argv)>1 else 'approach'
tgt=float(sys.argv[2]) if len(sys.argv)>2 else 331.0
lg(f'PERIM start mode={mode} tgt={tgt}')
tlast=0
while True:
    import statistics as _st
    ss=[b.sc(),b.sc(),b.sc()]
    s=[_st.median(v) for v in zip(*ss)]
    h=b.heading()
    if h is None: continue
    checks()
    if time.time()-tlast>4:
        tlast=time.time()
        lg(f'P ({X:.2f},{Y:.2f}) h={h:.0f} f={s[0]:.2f} l={s[4]:.2f} b={s[8]:.2f} r={s[12]:.2f} sig={b.p[6].last}')
        b.radio_send(f'A perimeter sweep of maze outside. sig={b.p[6].last}')
    if mode=='approach':
        e=angdiff(tgt,h)
        w=max(min(1.5*e,40),-40)
        v=0.55 if s[0]>1.5 else 0.3
        if s[0]<0.75 and min(s[1],s[15])<1.5:
            lg(f'ARRIVED at structure pose=({X:.2f},{Y:.2f})'); mode='follow'
            continue
        # dodge slight obstacles
        if s[1]<0.4 or s[2]<0.4: w-=25
        if s[15]<0.4 or s[14]<0.4: w+=25
        drive(v,w,h)
    else:
        # wall on RIGHT (beam12). keep dist ~0.3
        d=s[12]; dfr=min(s[13],s[14]); f=min(s[0],s[15])
        if f<0.32:
            # corner: turn left in place
            drive(0,0,h); b.wheels(-20,20); time.sleep(0.25); b.stop(); lastv=0
            continue
        if d>1.4 and dfr>1.4 and f>1.4:
            lost=globals().get('_lost',0)+1
            globals()['_lost']=lost
            if lost>25:
                lg('LOST wall -> approach'); mode='approach'; globals()['_lost']=0
                continue
        else: globals()['_lost']=0
        e=(min(d,1.0)-0.32)
        w=max(min(-e*90,35),-35)
        if dfr<0.25: w+=20
        v=0.42 if f>0.7 else 0.22
        if b.stalled():
            lg('stall'); b.wheels(-70,-70); time.sleep(0.5); b.stop()
            b.wheels(-20,20); time.sleep(0.4); b.stop(); lastv=0
            continue
        drive(v,w,h)
    time.sleep(0.07)
