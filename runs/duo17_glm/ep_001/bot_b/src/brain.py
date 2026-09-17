import os, select, time, sys, math, json

LOG = open('/memory/brain.log','a',buffering=1)
def log(s): LOG.write(f"{time.strftime('%H:%M:%S')} {s}\n")

def rdp(p, to=0.04):
    fd=os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],to)
    v=None
    if r:
        try: v=os.read(fd,4096).decode().strip()
        except: v=None
    os.close(fd)
    return v
def wrp(p,val):
    try:
        fd=os.open('/dev/robot/'+p, os.O_WRONLY)
        os.write(fd,(str(val)+'\n').encode())
        os.close(fd)
    except Exception as e:
        log(f"WRERR {p} {e}")

state = {'L':0.0,'R':0.0,'cmdL':0,'cmdR':0,'lastping':0,'pings':0,'start':time.time()}
def motors(l,r):
    # clamp
    l=max(-2.0,min(2.0,l)); r=max(-2.0,min(2.0,r))
    if abs(l-state['cmdL'])>0.01: wrp('d1',round(l,2)); state['cmdL']=l
    if abs(r-state['cmdR'])>0.01: wrp('d7',round(r,2)); state['cmdR']=r

def sense():
    s={}
    v=rdp('d2')
    if v:
        try: s['d2']=[float(x) for x in v.split(',')]
        except: pass
    for p in ['d3','d4','d0','d5','d6','d9','d11','d10']:
        v=rdp(p)
        if v: s[p]=v
    return s

def beam_angle(i): return math.radians(-90+12*i)  # CW positive from nose

def decide(s):
    prof=s.get('d2')
    if not prof or len(prof)!=16: return  # no data, keep old cmd
    # clear invalid beams: treat -1 as 1.5 (assume open? no—unknown). Use 0.3 (assume blocked-ish, cautious)
    P=[x if x>=0 else 0.3 for x in prof]
    # steering: weighted gap following
    front=min(P[6:10])
    best_i=max(range(16), key=lambda i:P[i])
    best_v=P[best_i]
    if front<0.30:
        # emergency turn toward more open side
        left_open=max(P[10:16]); right_open=max(P[0:6])
        if left_open>right_open: motors(-0.8, 0.8)   # CCW
        else: motors(0.8, -0.8)                       # CW
        return
    # desired relative angle = angle of best beam, but blend toward forward
    tgt=beam_angle(best_i)
    if best_v<0.5:
        # all beams short: spin toward widest
        tgt=beam_angle(best_i)
        motors(0.8 if math.sin(tgt)>0 else -0.8, -0.8 if math.sin(tgt)>0 else 0.8)
        return
    steer=math.degrees(tgt)/45.0  # -2..2
    steer=max(-1.2,min(1.2,steer))
    sp=1.2 if front>0.6 else 0.7
    motors(sp+steer*0.4, sp-steer*0.4)
    # sign: positive steer = target right (CW) => left wheel faster? CW turn = left faster.
    # steer>0 means target is to the RIGHT (angle CW positive). Turning right (CW): left wheel FASTER.
    # motors(l=sp+|..|)... fix below if wrong.

def main():
    log("brain started")
    n=0
    while True:
        n+=1
        s=sense()
        try:
            L=float(s.get('d6','nan')); R=float(s.get('d9','nan'))
            state['L'],state['R']=L,R
        except: pass
        decide(s)
        # radio housekeeping every ~8 s
        if time.time()-state['lastping']>8:
            state['lastping']=time.time()
            wrp('d8', f"A:{state['pings']}:status?")
            state['pings']+=1
            tx=(s.get('d3') or '')
            log(f"PING tx={tx} d0={s.get('d0')} d5={s.get('d5')} d11={s.get('d11')}")
        if n%10==0:
            h=s.get('d4'); 
            log(f"n={n} h={h} enc=({state['L']:.0f},{state['R']:.0f}) d2={s.get('d2')}")
        time.sleep(0.25)

main()
