import time, math, select
D='/dev/robot/'
LOG=open('/bot/src/log3.txt','a')
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
                if len(v)==16: return v
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
def main():
    b=Bot(); last_tx=0; last_stat=0; last_map=0
    mode='fwd'; turndir=1
    log('=== ctrl3 wallfollow start ===')
    while True:
        b.upd()
        l=lidar()
        if l is None:
            drive(0,0); time.sleep(0.2); continue
        l=[x if x>0 else 3.0 for x in l]
        front=min(l[0],l[1],l[15])
        now=time.time()
        if mode=='fwd':
            if front<0.28:
                mode='turn'
                # right-wall rule: turn left (heading + => beam4 side)
                turndir=1
                drive(34,-34)
            else:
                err=min(l[12],l[13])-0.35
                err=max(-0.25,min(0.25,err))
                turn=int(err*120)
                sp=60 if front>0.6 else 35
                drive(sp-turn, sp+turn)
        else:
            if l[0]>0.55 and front>0.3: mode='fwd'
            else: drive(34*turndir,-34*turndir)
        d5=rdf('d5'); st=rd('d6')
        here='here=1' in (st or '')
        if here:
            log(f'*** HERE=1 at ({b.x:.2f},{b.y:.2f}) st={st}')
            drive(0,0)
            wr('d0','GOAL FOUND by botA! I am standing on it.')
            time.sleep(5)
        if now-last_tx>3:
            wr('d0', f'HELLO botA exploring x={b.x:.2f} y={b.y:.2f} d5={d5}')
            last_tx=now
        if now-last_stat>2:
            log(f'pos=({b.x:.2f},{b.y:.2f}) h={b.h:.0f} {st} d5={d5} d2={rd("d2")} d9={rd("d9")} m={mode}')
            last_stat=now
        if now-last_map>0.5:
            # record obstacles in world frame
            pts=[]
            for k in range(16):
                r=l[k]
                if 0<r<2.4:
                    a=math.radians(b.h+22.5*k)
                    pts.append(f'{b.x+r*math.cos(a):.2f},{b.y+r*math.sin(a):.2f}')
            print(f'{b.x:.2f},{b.y:.2f}|'+';'.join(pts), file=MAP); MAP.flush()
            last_map=now
        time.sleep(0.05)
main()
