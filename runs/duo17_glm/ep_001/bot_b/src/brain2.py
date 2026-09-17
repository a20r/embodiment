import os, select, time, math, sys

LOG=open('/memory/brain2.log','a',buffering=1)
def log(s): LOG.write(f"[{time.strftime('%H:%M:%S')}] {s}\n")

def rd(p,to=0.04):
    fd=os.open('/dev/robot/'+p,os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],to)
    v=None
    if r:
        try: v=os.read(fd,4096).decode().strip()
        except: v=None
    os.close(fd); return v
def wr(p,v):
    try:
        fd=os.open('/dev/robot/'+p,os.O_WRONLY)
        os.write(fd,(str(v)+'\n').encode()); os.close(fd)
    except Exception as e: log(f"WRERR{p}:{e}")

# ---- state ----
cmdL=0.0; cmdR=0.0
x=y=0.0; heading=None; lastE=(None,None)
path=[]          # (t,x,y,heading)
pings=0; lastPing=0
radio_msgs=[]
mode="explore"; modeUntil=0
bumps=0
t0=time.time()

def motors(l,r):
    global cmdL,cmdR
    l=max(-2,min(2,l)); r=max(-2,min(2,r))
    if abs(l-cmdL)>0.02: wr('d1',round(l,2)); cmdL=l
    if abs(r-cmdR)>0.02: wr('d7',round(r,2)); cmdR=r

def beam_rel(i): return math.radians(22.5*(i-8))  # 0=ahead, +CW

def update_deadreckon(e6,e9):
    global lastE,x,y
    if lastE[0] is None: lastE=(e6,e9); return
    dR=e6-lastE[0]; dL=e9-lastE[1]
    lastE=(e6,e9)
    # wrap unwinding: encoders are cumulative ints, fine
    d=(dR+dL)/2.0  # units ~cm
    if heading is not None:
        # compass: 0=N?, x=east?, assume meters: d cm -> m
        hrad=math.radians(heading)
        x+= (d/100.0)*math.sin(hrad)
        y+= (d/100.0)*math.cos(hrad)

def sense():
    s={}
    for p in ['d2','d3','d4','d0','d5','d6','d9','d11','d10']:
        v=rd(p)
        if v: s[p]=v
    return s

def best_dir(P):
    # prefer forward openness: score beams idx6..10 heavy, plus sides for escapes
    scores={}
    for i in range(16):
        if P[i]<0: P[i]=0.25
    # weighted: openness ahead
    fwd=max(P[7],P[8],P[9])
    # choose target beam: highest distance among idx 5..11 (forwardish), fallback any
    cand=range(5,12)
    bi=max(cand,key=lambda i:P[i])
    # if very blocked ahead, consider all
    if fwd<0.45:
        bi=max(range(16),key=lambda i:P[i])
    return bi,P[bi]

def drive(P,sp_base=1.2):
    bi,v=best_dir(P)
    ang=math.degrees(beam_rel(bi))  # -180..180, CW+
    steer=max(-1.0,min(1.0,ang/40.0))
    # clearance assist
    left=min([P[i] for i in range(3,9)]); right=min([P[i] for i in range(9,14)])
    if left<0.22: steer+= (0.22-left)*2.5     # push right (steer+ = CW = right)
    if right<0.22: steer-= (0.22-right)*2.5
    steer=max(-1.4,min(1.4,steer))
    sp=sp_base if min([v for v in P[7:10] if v>=0] or [0])>0.45 else 0.7
    l=sp+steer*0.5; r=sp-steer*0.5
    motors(l,r)

def main():
    global heading,lastPing,pings,bumps,mode,modeUntil
    log("brain2 start")
    n=0
    while True:
        n+=1
        s=sense()
        try:
            heading=float(s['d4'])
        except: pass
        try:
            e6=float(s['d6']); e9=float(s.get('d9'))
            update_deadreckon(e6,e9)
        except: pass
        P=None
        if 'd2' in s:
            try: P=[float(v) for v in s['d2'].split(',')]
            except: P=None
        if len(path)==0 or time.time()-path[-1][0]>4:
            path.append((round(time.time()-t0,1),round(x,2),round(y,2),heading))
        bump = (s.get('d0')=='1') or (s.get('d5')=='1')
        if bump: bumps+=1
        now=time.time()
        if now<modeUntil:
            pass # continue current avoidance maneuver
        elif bump or (P and min([v for v in P[8:10] if v>=0] or [9])<0.18):
            # too close / bump: back up and turn away from close side
            frontv = min([v for v in P[8:10] if v>=0] or [9]) if P else 9
            log(f"AVOID bump={bump} front={frontv} P={P}")
            lClose = P and min([v for v in P[3:8] if v>=0] or [9])<0.15
            rClose = P and min([v for v in P[9:14] if v>=0] or [9])<0.15
            if lClose and not rClose:
                motors(-0.7,0.7); mode="avoidL"; modeUntil=now+1.2
            elif rClose and not lClose:
                motors(0.7,-0.7); mode="avoidR"; modeUntil=now+1.2
            else:
                motors(-0.8,-0.8); mode="back"; modeUntil=now+0.8
        elif P:
            drive(P)
        # radio
        if now-lastPing>5:
            lastPing=now; pings+=1
            wr('d8',f"A{pings}:A-robot,report")
            d10=s.get('d10')
            if d10: log(f"RADIO RX: {d10}"); radio_msgs.append(d10)
        if n%12==0:
            log(f"n={n} h={heading} xy=({x:.2f},{y:.2f}) enc=({s.get('d6')},{s.get('d9')}) d0={s.get('d0')} d5={s.get('d5')} d11={s.get('d11')} d3={s.get('d3')}")
            log(f"   P={s.get('d2')}")
        time.sleep(0.22)

main()
