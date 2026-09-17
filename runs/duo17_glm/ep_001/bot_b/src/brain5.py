import os, select, time, math, random
LOG=open('/memory/brain5.log','a',buffering=1)
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

def P():
    v=rd('d2')
    try:
        p=[float(z) for z in v.split(',')]
        return [z if z>=0 else 1.45 for z in p]
    except: return None

def bump():
    return (rd('d0')=='1') or (rd('d5')=='1')

x=y=0.0; heading=None; lastE=[None,None]
def dr():
    global x,y,heading
    h=rdF('d4')
    if h is not None: heading=h
    e6,e9=rdF('d6'),rdF('d9')
    if e6 is None or e9 is None: return
    if lastE[0] is not None:
        dR=e6-lastE[0]; dL=e9-lastE[1]
        if abs(dR)<300 and abs(dL)<300:
            d=(dR+dL)/2.0
            hh=math.radians(heading or 0)
            x+=(d/1000.0)*math.sin(hh); y+=(d/1000.0)*math.cos(hh)
    lastE[0]=e6; lastE[1]=e9

pings=0; lastPing=0; t0=time.time(); stuck=0
rotDir=0; rotUntil=0
prevBi=1
lastTx='?'
def radio():
    global pings,lastPing,lastTx
    if time.time()-lastPing<1.2: return
    lastPing=time.time(); pings+=1
    wr('d8',f"A{pings}:beacon")
    rx=rd('d10')
    if rx: log(f"RADIO RX: {rx}")
    d3=rd('d3') or ''
    tx=d3.split('tx=')[-1].split(':')[-1] if 'tx=' in d3 else '?'
    if tx!=lastTx:
        log(f"TXSTATUS {d3} P={rd('d2')}")
        lastTx=tx
    if pings%40==0:
        log(f"STAT h={heading} xy=({x:.2f},{y:.2f}) d11={rd('d11')} {d3}")
    if 'goal=1' in d3 or 'here=1' in d3:
        log(f"FLAG! {d3}")

def align():
    """rotate (committed) until max beam within +/-1 idx of nose(idx1); True when aligned"""
    global rotDir, rotUntil, prevBi
    Pc=P()
    if not Pc: return False
    S=[sum(Pc[(i-1)%16:i+2])/3.0 for i in range(16)]  # smoothed
    # hysteresis bonus toward previous target
    global prevBi
    for i in range(16):
        dd=abs((i-prevBi+16)%16); dd=min(dd,16-dd)
        if dd<=2: S[i]+=0.06
    bi=max(range(16),key=lambda i:S[i])
    d=(bi-1)%16
    if d>8: d-=16
    if abs(d)<=1 and Pc[1]>0.35:
        return True
    if abs(d)<=2 and Pc[1]>0.5 and Pc[0]>0.4 and Pc[2]>0.4:
        return True  # good enough, keep driving
    now=time.time()
    if now<rotUntil and rotDir!=0:
        # continue committed rotation, refresh target each eval
        if rotDir>0: wr('d1','-2'); wr('d7','2')
        else: wr('d1','2'); wr('d7','-2')
        return False
    prevBi=bi
    rotDir=1 if d<0 else -1
    dur=max(1.5,abs(d)*6.4*0.55)
    rotUntil=now+dur
    log(f"ALIGN commit: best={bi}(S={S[bi]:.2f}) d={d} rotate {'CCW' if rotDir>0 else 'CW'} {dur:.1f}s")
    if rotDir>0: wr('d1','-2'); wr('d7','2')
    else: wr('d1','2'); wr('d7','-2')
    return False

def main():
    global stuck
    log("brain5 start")
    state='align'
    tState=time.time()
    while True:
        dr(); radio()
        now=time.time()
        if state=='align':
            if align():
                wr('d1','0'); wr('d7','0'); time.sleep(0.4)
                state='tap'; tState=now; log("ALIGNED nose -> tap fwd")
            elif now-tState>30:
                # worst case: rotate ~90 more and re-eval next loop
                tState=now
        elif state=='tap':
            Pc=P()
            if Pc and min(Pc[0],Pc[1],Pc[2])>0.55:
                wr('d1','2'); wr('d7','2')
            elif Pc and min(Pc[0],Pc[1],Pc[2])>0.4:
                wr('d1','1'); wr('d7','1')
            else:
                wr('d1','1'); wr('d7','1')
            if bump():
                wr('d1','-1'); wr('d7','-1'); state='back'; tState=now
                log(f"GRIND during tap (xy={x:.2f},{y:.2f}) back off")
            elif now-tState>(14 if Pc and min(Pc[0],Pc[1],Pc[2])>0.55 else 8):
                wr('d1','0'); wr('d7','0'); time.sleep(0.4)
                state='align'; tState=now
                log(f"tap done h={heading:.0f} xy=({x:.2f},{y:.2f}) d11={rd('d11')} P={rd('d2')}")
        elif state=='back':
            if now-tState>1.6:
                wr('d1','0'); wr('d7','0'); time.sleep(0.3)
                state='turn'; tState=now
        elif state=='turn':
            # rotate ~50 deg away; alternate direction
            side=1 if stuck%2==0 else -1
            if side>0: wr('d1','2'); wr('d7','-2')
            else: wr('d1','-2'); wr('d7','2')
            if now-tState>7.5:
                wr('d1','0'); wr('d7','0'); time.sleep(0.3)
                stuck+=1; state='align'; tState=now
                log(f"turned away; h={heading:.0f}")
        time.sleep(0.18)

main()
