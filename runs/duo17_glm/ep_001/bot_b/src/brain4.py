import os, select, time, math
LOG=open('/memory/brain4.log','a',buffering=1)
def log(s): LOG.write(f"[{time.strftime('%H:%M:%S')}] {s}\n")
def rd(p,to=0.04):
    fd=os.open('/dev/robot/'+p,os.O_RDONLY|os.O_NONBLOCK)
    r,_,_=select.select([fd],[],[],to)
    v=None
    if r:
        try: v=os.read(fd,4096).decode().strip()
        except: v=None
    os.close(fd); return v
def rdF(p):
    v=rd(p)
    try: return float(v)
    except: return None
def wr(p,v):
    try:
        fd=os.open('/dev/robot/'+p,os.O_WRONLY)
        os.write(fd,(str(v)+'\n').encode()); os.close(fd)
    except Exception as e: log(f"WRERR{p}:{e}")

# integer maneuvers (l,r)
MAN={'F':(1,1),'FR':(2,1),'HR':(2,0),'SR':(2,-1),'FL':(1,2),'HL':(0,2),'SL':(-1,2),
     'B':(-1,-1),'CW':(2,-2),'CCW':(-2,2),'STOP':(0,0)}
cur=None
def man(name):
    global cur
    if cur!=name:
        l,r=MAN[name]; wr('d1',l); wr('d7',r); cur=name
        log(f"MAN {name} (l={l},r={r})")

x=y=0.0; heading=None; lastE=[None,None]; t0=time.time()
pings=0; lastPing=0; bumpT=0; lastLog=0
stats={'F':0,'FR':0,'FL':0,'B':0,'CW':0,'CCW':0,'STOP':0,'HR':0,'HL':0,'SR':0,'SL':0}
def dr():
    global x,y
    e6,e9=rdF('d6'),rdF('d9')
    if e6 is None or e9 is None: return
    if lastE[0] is not None:
        dR=e6-lastE[0]; dL=e9-lastE[1]
        if abs(dR)<300 and abs(dL)<300:
            d=(dR+dL)/2.0
            h=math.radians(heading or 0)
            x+=(d/1000.0)*math.sin(h); y+=(d/1000.0)*math.cos(h)
    lastE[0]=e6; lastE[1]=e9

def P():
    v=rd('d2')
    try:
        p=[float(z) for z in v.split(',')]
        return [z if z>=0 else 2.0 for z in p]  # invalid -> treat open (corridor)
    except: return None

def decide(Pc):
    # returns maneuver name
    bump=(rd('d0')=='1') or (rd('d5')=='1')
    if bump: return 'B'
    f7,f8,f9,f10=Pc[7],Pc[8],Pc[9],Pc[10]
    front=min(f8,f9)
    if front<0.22: return 'B'
    # pick target among forwardish
    bi=max(range(6,12),key=lambda i:Pc[i])
    # corner avoidance: f7 very close -> bear right; f10 very close -> bear left
    if f7<0.16: bi=max(bi,10)
    if f10<0.16: bi=min(bi,7)
    if bi<=7: return 'FL'
    if bi==8: return 'F' if min(Pc[6:11])>0.35 else 'F'
    if bi==9: return 'F'
    return 'FR'

def main():
    global heading,pings,lastPing,cur
    log("brain4 start")
    phase='drive'; phaseUntil=0
    n=0
    while True:
        n+=1
        now=time.time()
        if 'd4v' not in dir(): pass
        hv=rdF('d4')
        if hv is not None: heading=hv
        dr()
        Pc=P()
        bump=(rd('d0')=='1') or (rd('d5')=='1')
        # radio
        if now-lastPing>5:
            lastPing=now; pings+=1
            wr('d8',f"A{pings} ping")
            rx=rd('d10')
            if rx: log(f"RADIO RX: {rx}")
            d3=rd('d3')
            if d3 and ('goal=1' in d3 or 'here=1' in d3): log(f"FLAG! {d3}")
        # state machine: drive 12s, rest 4s (energy), plus bump handling
        if now<phaseUntil:
            time.sleep(0.15); continue
        if phase=='drive':
            # end of burst -> rest
            man('STOP'); phase='rest'; phaseUntil=now+4
            if n%2==0 or True:
                log(f"burst end h={heading:.1f} xy=({x:.2f},{y:.2f}) e11={rd('d11')} P={rd('d2')}")
        elif phase=='rest':
            if bump:
                log(f"bump during rest d0={rd('d0')} d5={rd('d5')}")
                man('B'); phase='drive'; phaseUntil=now+1.2
            else:
                Pc=P()
                if Pc:
                    m=decide(Pc)
                    stats[m]=stats.get(m,0)+1
                    man(m)
                phase='drive'; phaseUntil=now+12
        time.sleep(0.15)

main()
