import time, math, select, random, collections
D='/dev/robot/'
LOG=open('/bot/src/log4.txt','a')
MAP=open('/bot/src/map.txt','a')
def log(*a):
    print(time.strftime('%H:%M:%S'), *a, file=LOG); LOG.flush()
def rd(p, timeout=0.5):
    try:
        f=open(D+p)
        r,_,_=select.select([f],[],[],timeout)
        if not r: f.close(); return None
        line=f.readline().strip(); f.close(); return line
    except: return None
def wr(p,s):
    try:
        with open(D+p,'w') as f: f.write(str(s)+'\n')
    except: pass
def lidar():
    for _ in range(3):
        l=rd('d3')
        if l:
            try:
                v=[float(x) for x in l.split(',')]
                if len(v)==16: return [x if x>0 else 3.0 for x in v]
            except: pass
    return None
def rdf(p):
    for _ in range(3):
        v=rd(p)
        if v:
            try: return float(v)
            except: pass
    return None
def drive(l,r): wr('d10',int(l)); wr('d11',int(r))
SCALE=0.001
class Bot:
    def __init__(self):
        self.x=0.0; self.y=0.0; self.h=rdf('d1') or 0.0
        self.el=rdf('d7') or 0.0; self.er=rdf('d8') or 0.0
    def upd(self):
        h=rdf('d1'); el=rdf('d7'); er=rdf('d8')
        if h is not None: self.h=h
        if el is not None and er is not None:
            dl=el-self.el; dr=er-self.er; self.el=el; self.er=er
            d=SCALE*(dl+dr)/2
            if abs(d)<1.0:
                rad=math.radians(self.h)
                self.x+=d*math.cos(rad); self.y+=d*math.sin(rad)
visits=collections.Counter()
def cell(x,y): return (round(x/0.3), round(y/0.3))
b=Bot()
def check_events(st):
    if st and 'here=1' in st:
        log(f'*** HERE=1 at ({b.x:.2f},{b.y:.2f}) {st}')
        drive(0,0)
        for _ in range(3): wr('d0','GOAL FOUND by botA, here=1! staying put.')
        return True
    return False
def turn_to(target_h, tol=9, tmax=6):
    t0=time.time()
    while time.time()-t0<tmax:
        b.upd()
        dh=(target_h-b.h+180)%360-180
        if abs(dh)<tol: drive(0,0); return True
        s=max(14,min(36,int(abs(dh)*0.7)))
        drive(s if dh>0 else -s, -s if dh>0 else s)
        time.sleep(0.05)
    drive(0,0); return False
def leg(target_h, maxdist=1.5, sp=55):
    d0=(b.el+b.er)/2
    last_stat=0
    while True:
        b.upd()
        l=lidar()
        if not l: continue
        f=min(l[0],l[1],l[15])
        visits[cell(b.x,b.y)]+=1
        st=rd('d6')
        if check_events(st): return 'GOAL'
        now=time.time()
        if now-last_stat>2:
            log(f'pos=({b.x:.2f},{b.y:.2f}) h={b.h:.0f} {st} d5={rdf("d5")} f={f:.2f}')
            wr('d0', f'HELLO botA exploring x={b.x:.2f} y={b.y:.2f}')
            last_stat=now
            pts=[]
            for k in range(16):
                r=l[k]
                if 0<r<2.4:
                    a=math.radians(b.h+22.5*k)
                    pts.append(f'{b.x+r*math.cos(a):.2f},{b.y+r*math.sin(a):.2f}')
            print(f'{b.x:.2f},{b.y:.2f}|'+';'.join(pts), file=MAP); MAP.flush()
        if f<0.30: drive(0,0); return 'blocked'
        if (b.el+b.er)/2-d0>maxdist/SCALE: drive(0,0); return 'dist'
        bal=0.0
        if min(l[4],l[12])<0.45: bal=(l[4]-l[12])*50
        dh=(target_h-b.h+180)%360-180
        turn=max(-30,min(30,dh*1.0+bal))
        spd=sp if f>0.55 else 32
        drive(int(spd+turn),int(spd-turn))
        time.sleep(0.07)
def choose():
    l=lidar()
    if not l: return None
    best=None; bestsc=-1e9
    for k in range(16):
        r=min(l[k], l[(k+1)%16]+0.35, l[(k-1)%16]+0.35)
        if r<0.45: continue
        ang=math.radians(b.h+22.5*k)
        look=min(r-0.15,0.9)
        cx=b.x+look*math.cos(ang); cy=b.y+look*math.sin(ang)
        v=visits[cell(cx,cy)]
        sc=min(r,1.5) - 1.2*min(v,6) + random.random()*0.3
        if sc>bestsc: bestsc=sc; best=k
    if best is None: return None
    return (b.h+22.5*best)%360
log('=== explore2 start ===')
while True:
    t=choose()
    if t is None:
        drive(-30,-30); time.sleep(1); drive(0,0); continue
    turn_to(t)
    r=leg(t)
    if r=='GOAL':
        while True:
            wr('d0','GOAL FOUND by botA, here=1. climb d5 to find me.')
            log('at goal, waiting. '+str(rd('d6')))
            time.sleep(5)
