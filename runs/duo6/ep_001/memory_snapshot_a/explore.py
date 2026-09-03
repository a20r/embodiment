import time, math, json, threading
DEV='/dev/robot/'
def rd(p):
    try:
        with open(DEV+p) as f: return f.readline().strip()
    except: return ''
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
apos=[None]
def rx_loop():
    while True:
        s=rd('d4')
        if s:
            try:
                d={kv.split('=')[0]:float(kv.split('=')[1]) for kv in s.split() if '=' in kv}
                if 'x' in d: apos[0]=(d.get('t'),d['x'],d['y']); log({'a':[d.get('t'),d['x'],d['y'],d.get('h')]})
                else: log({'rx':s})
            except: log({'rx':s})
        else: time.sleep(0.2)
threading.Thread(target=rx_loop,daemon=True).start()
K=3.6e-4
x=y=0.0
e7p=rdn('d7'); e8p=rdn('d8')
side=-1
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
    d6=rd('d6'); goal=('goal' in d6) and ('goal=0' not in d6)
    if goal:
        drive(0,0)
        wr('d0',f'ATGOAL x={x:.2f} y={y:.2f}')
        log({'st':'GOAL','d6':d6,'x':round(x,2),'y':round(y,2)})
        time.sleep(1); continue
    front=mn([l[15],l[0],l[1]])
    sd=mn([l[11],l[12],l[13]]) if side<0 else mn([l[3],l[4],l[5]])
    away=1 if side<0 else -1
    st='follow'
    if mn(l)<0.05:
        i=l.index(mn(l)); b=-50 if (i<=3 or i>=13) else 50
        drive(b,0); time.sleep(0.5); st='esc'
    elif front<0.20:
        drive(0,away*55); st="turn"
    elif sd>0.45:
        drive(55,-away*35); st="arc"
    else:
        err=sd-0.18
        drive(85,-away*max(-35,min(35,err*160)))
    if now-last_flip>1800:
        side=-side; last_flip=now; log({'flip':side})
    if now-last_tx>2:
        wr('d0',f'HELLO from bot B t={int(now)} x={x:.2f} y={y:.2f} h={int(h) if h else 0}'); last_tx=now
    if now-last_log>2:
        log({'st':st,'x':round(x,2),'y':round(y,2),'h':h,'d5':d5,'f':round(front,2),'d2':rd('d2'),'d6':d6,'l':l}); last_log=now
    time.sleep(0.2)
finally:
  drive(0,0)
