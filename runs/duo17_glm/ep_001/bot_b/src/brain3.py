import os, select, time, math

LOG=open('/memory/brain3.log','a',buffering=1)
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

# integer maneuvers: (left,right)
M={
 'FWD':(1,1),'FAST':(2,2),
 'SOFT_R':(2,1),'HARD_R':(2,0),'SPIN_R':(1,-1),
 'SOFT_L':(1,2),'HARD_L':(0,2),'SPIN_L':(-1,1),
 'BACK':(-1,-1),'STOP':(0,0),
}
cur=('STOP',0,0)
def man(name,dur=0.0):
    """set maneuver; dur>0 means hold at least dur sec before re-eval"""
    global cur
    if cur[0]!=name:
        l,r=M[name]; wr('d1',l); wr('d7',r); cur=(name,time.time(),dur)
        return True
    return False

x=y=0.0; heading=None; lastE=[None,None]
path=[]; pings=0; lastPing=0; t0=time.time()
holdUntil=0; lastAvoid=0; bumps=0; junctions=[]

def dr(e6,e9):
    global x,y
    if lastE[0] is not None:
        dR=e6-lastE[0]; dL=e9-lastE[1]
        if abs(dR)<400 and abs(dL)<400:
            d=(dR+dL)/2.0
            h=math.radians(heading or 0)
            x+=(d/100.0)*math.sin(h); y+=(d/100.0)*math.cos(h)
    lastE[0]=e6; lastE[1]=e9

def clean(P):
    return [v if v>=0 else 0.3 for v in P]

def decide(P,bump):
    global holdUntil,lastAvoid
    now=time.time()
    P=clean(P)
    front=min(P[8],P[9]); f7=P[7]; f10=P[10]
    # emergencies
    if bump or front<0.18 or f7<0.10 or f10<0.10:
        if now-lastAvoid>1.5:
            log(f"AVOID bump={bump} front={front:.2f} P={P}")
            lastAvoid=now
        if bump or front<0.18:
            man('BACK'); holdUntil=now+0.8; return
        # wall corner at 45deg: turn away
        if f7<0.10: man('SPIN_R'); holdUntil=now+0.5; return
        if f10<0.10: man('SPIN_L'); holdUntil=now+0.5; return
    if now<holdUntil: return  # hold current maneuver
    # target selection
    bi=max(range(5,12),key=lambda i:P[i])
    if front<0.45:
        bi=max(range(16),key=lambda i:P[i])
    ang=22.5*(bi-8.5)
    # clearance bias
    left=min(P[3:8]); right=min(P[9:14])
    if left<0.20: ang=max(ang,25)
    if right<0.20: ang=min(ang,-25)
    if ang<-35: man('SPIN_L')
    elif ang<-12: man('SOFT_L')
    elif ang<=12: man('FWD')
    elif ang<=35: man('SOFT_R')
    else: man('SPIN_R')

def main():
    global heading,pings,lastPing,bumps,holdUntil
    log("brain3 start")
    n=0
    while True:
        n+=1
        s={}
        for p in ['d2','d3','d4','d0','d5','d6','d9','d11','d10']:
            v=rd(p)
            if v: s[p]=v
        if 'd4' in s:
            try: heading=float(s['d4'])
            except: pass
        if 'd6' in s and 'd9' in s and heading is not None:
            try: dr(float(s['d6']),float(s['d9']))
            except: pass
        P=None
        if 'd2' in s:
            try: P=[float(v) for v in s['d2'].split(',')]
            except: P=None
        bump=(s.get('d0')=='1')or(s.get('d5')=='1')
        if bump: bumps+=1
        if P and len(P)==16 and heading is not None:
            decide(P,bump)
        if time.time()-path[-1][0] if path else True: pass
        if not path or time.time()-path[-1][0]>4:
            path.append((round(time.time()-t0,1),round(x,2),round(y,2),heading))
        # junction logging: side openings
        if P and len(P)==16:
            Pc=clean(P)
            if Pc[3]>1.2 or Pc[13]>1.2:
                node=(round(x,1),round(y,1),int(heading or 0),'L' if Pc[3]>1.2 else 'R')
                if not junctions or junctions[-1][:3]!=node[:3]:
                    junctions.append(node); log(f"JUNCTION? {node} P={P}")
        # radio
        if time.time()-lastPing>5:
            lastPing=time.time(); pings+=1
            wr('d8',f"A{pings} PINGA status?")
            if s.get('d10'): log(f"RADIO RX: {s['d10']}")
        if n%15==0:
            log(f"n={n} m={cur[0]} h={heading} xy=({x:.2f},{y:.2f}) enc=({s.get('d6')},{s.get('d9')}) d0={s.get('d0')} d5={s.get('d5')} d11={s.get('d11')} d3={(s.get('d3') or '')[-12:]}")
        time.sleep(0.2)

main()
