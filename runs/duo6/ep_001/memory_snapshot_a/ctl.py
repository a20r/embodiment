import time, math, json, os, sys, threading

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

# radio RX thread
rx_msgs=[]
def rx_loop():
    while True:
        s=rd('d4')
        if s:
            rx_msgs.append(s)
            log({'rx':s})
        else:
            time.sleep(0.5)
threading.Thread(target=rx_loop,daemon=True).start()

K_ENC=3.6e-4
x,y=0.0,0.0
e7p=rdn('d7'); e8p=rdn('d8')
side_sign=-1   # follow wall at ray12 (heading-90); diff>0 turns toward ray4
last_tx=0; last_log=0
state='follow'
goal=False
t_start=time.time()

def drive(base,diff):
    diff=max(-60,min(60,diff))
    wr('d10',base+diff/2); wr('d11',base-diff/2)

try:
  while True:
    l=lid()
    if not l: time.sleep(0.2); continue
    h=rdn('d1')
    e7=rdn('d7'); e8=rdn('d8')
    if h is not None and e7 is not None and e8 is not None and e7p is not None:
        d=((e7-e7p)+(e8-e8p))/2*K_ENC
        x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
        e7p,e8p=e7,e8
    d6=rd('d6')
    goal = 'goal=1' in d6 or 'goal=0' not in d6 and 'goal' in d6 and not d6.endswith('goal=0')
    goal = 'goal=0' not in d6 if 'goal' in d6 else goal
    front=mn([l[15],l[0],l[1]])
    if side_sign<0:
        side=mn([l[11],l[12],l[13]]); opens=mn([l[12]])
    else:
        side=mn([l[3],l[4],l[5]]); opens=mn([l[4]])
    away=side_sign*(-1)  # diff sign to turn away from wall... diff>0 -> toward ray4
    # turn away from wall: if wall at ray12 (side_sign=-1), away = +
    away = 1 if side_sign<0 else -1
    now=time.time()
    if goal:
        drive(0,0); state='goal'
        if now-last_tx>3:
            wr('d0',f'ATGOAL x={x:.2f} y={y:.2f}'); last_tx=now
        if now-last_log>5: log({'st':'GOAL','d6':d6,'x':round(x,2),'y':round(y,2)}); last_log=now
        time.sleep(0.5); continue
    # emergency
    if mn(l)<0.05:
        i=l.index(mn(l))
        # back away: if obstacle in front half, reverse
        b=-50 if (i<=3 or i>=13) else 50
        drive(b,0); time.sleep(0.6); drive(0,0)
        state='escape'
    elif front<0.20:
        drive(0,away*45)   # rotate away from wall side
        state='turn'
    elif side>0.45:
        # opening on wall side: arc toward it
        drive(45,-away*30)
        state='arc'
    else:
        err=side-0.18
        steer=-away*max(-30,min(30,err*150))
        drive(60,steer)
        state='follow'
    if now-last_tx>5:
        wr('d0',f'PING bot x={x:.2f} y={y:.2f} h={h}'); last_tx=now
    if now-last_log>2:
        log({'st':state,'x':round(x,2),'y':round(y,2),'h':h,'f':round(front,2),'s':round(side,2),
             'd2':rd('d2'),'d5':rd('d5'),'d6':d6,'l':l})
        last_log=now
    time.sleep(0.25)
finally:
  drive(0,0)
