import time, math, json, threading
DEV='/dev/robot/'
def rd(p):
    try:
        with open(DEV+p) as f: return f.readline().strip()
    except Exception: return ''
def rdn(p):
    for _ in range(4):
        s=rd(p)
        if s:
            try: return float(s)
            except: pass
    return None
def wr(p,v):
    with open(DEV+p,'w') as f: f.write(f'{v}\n')
def lid():
    for _ in range(4):
        s=rd('d3')
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: return v
        except: pass
    return None
def mn(vals):
    v=[x for x in vals if x and x>0]
    return min(v) if v else 9.9
LOG=open('/memory/telemetry.jsonl','a')
def log(rec):
    rec['t']=round(time.time(),1)
    LOG.write(json.dumps(rec)+'\n'); LOG.flush()
AGOAL=[False]
def rx_loop():
    while True:
        s=rd('d4')
        if s:
            log({'rx':s})
            if ('GOALFOUND' in s or 'ATGOAL' in s) and 'bot B' not in s: AGOAL[0]=True
        else: time.sleep(0.25)
threading.Thread(target=rx_loop,daemon=True).start()
K=3.6e-4
x=y=0.0
e7p=rdn('d7'); e8p=rdn('d8')
side=-1  # -1: wall at ray12 side, +1: wall at ray4 side
hist=[]  # (t,d5)
last_tx=0; last_log=0; last_flip=time.time()
def drive(b,d):
    d=max(-70,min(70,d))
    wr('d10',b+d/2); wr('d11',b-d/2)
try:
  while True:
    l=lid()
    if not l: time.sleep(0.2); continue
    h=rdn('d1'); e7=rdn('d7'); e8=rdn('d8'); d5=rdn('d5')
    now=time.time()
    if None not in (h,e7,e8,e7p,e8p):
        dd=((e7-e7p)+(e8-e8p))/2*K
        x+=dd*math.cos(math.radians(h)); y+=dd*math.sin(math.radians(h))
        e7p,e8p=e7,e8
    if d5 is not None: hist.append((now,d5))
    hist=[p for p in hist if now-p[0]<6]
    d6=rd('d6'); goal=('goal=0' not in d6) and ('goal' in d6)
    if goal:
        drive(0,0)
        wr('d0','ATGOAL B flag=1 stopped, come to me via range')
        log({'st':'GOAL','d6':d6,'d5':d5}); time.sleep(1.5); continue
    front=mn([l[15],l[0],l[1]])
    if side<0: sd=mn([l[11],l[12],l[13]])
    else: sd=mn([l[3],l[4],l[5]])
    away=1 if side<0 else -1
    st='follow'
    # line-of-sight chase when close
    los=None
    if d5 is not None and d5<0.40:
        for k in range(16):
            nb=[l[(k-1)%16],l[(k+1)%16]]
            nb=[v for v in nb if v>0]
            if l[k]>0 and abs(l[k]-d5)<0.06 and l[k]<0.45 and nb and l[k]<min(nb)-0.12:
                los=k; break
    if mn(l)<0.05:
        i=l.index(mn(l)); b=-50 if (i<=3 or i>=13) else 50
        drive(b,0); time.sleep(0.5); st='esc'
    elif los is not None and d5>0.12:
        ang=los*22.5
        if ang>180: ang-=360
        if abs(ang)>30 and front<0.15:
            drive(0, 45 if ang>0 else -45)
        else:
            drive(45 if front>0.15 else 0, max(-50,min(50,ang*1.2)))
        st=f'los{los}'
    elif d5 is not None and d5<(0.05 if AGOAL[0] else 0.18):
        drive(0,0); st='near'
    elif front<0.20:
        drive(0,away*45); st='turn'
    elif sd>0.45:
        drive(45,-away*30); st='arc'
    else:
        err=sd-0.18
        drive(60,-away*max(-30,min(30,err*150)))
    # gradient: if moving away for a while, flip wall side
    if len(hist)>8 and now-last_flip>9:
        t0,v0=hist[0]; t1,v1=hist[-1]
        if v1-v0>0.045 and (d5 or 1)>0.25:
            side=-side; last_flip=now
            log({'flip':side,'d5':d5})
    if now-last_tx>4:
        wr('d0',f'B: tailing you, my d5={d5}'); last_tx=now
    if now-last_log>2:
        log({'st':st,'x':round(x,2),'y':round(y,2),'h':h,'d5':d5,'f':round(front,2),'sd':round(sd,2),'side':side,'d6':d6,'l':l}); last_log=now
    time.sleep(0.25)
finally:
  drive(0,0)
