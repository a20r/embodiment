import time, math, select

D='/dev/robot/'
LOG=open('/bot/src/log.txt','a')
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
    b=Bot(); last_tx=0; last_stat=0
    mode='fwd'; turndir=1
    log('=== ctrl2 start ===')
    while True:
        b.upd()
        l=lidar()
        if l is None:
            drive(0,0); time.sleep(0.2); continue
        l=[x if x>0 else 2.5 for x in l]
        front=min(l[0],l[1],l[15])
        if mode=='fwd':
            if front<0.28:
                mode='turn'
                # turn toward more open side: left(beam4, +h) vs right(beam12,-h)
                left=max(l[3],l[4],l[5]); right=max(l[11],l[12],l[13])
                turndir=1 if left>right else -1
                drive(35*turndir,-35*turndir)
            else:
                # right-wall follow on beam12
                err=min(l[12],l[13])-0.35
                err=max(-0.25,min(0.25,err))
                turn=int(err*120)
                sp=60 if front>0.6 else 35
                drive(sp-turn, sp+turn)
        else: # turn
            if l[0]>0.55 and front>0.3:
                mode='fwd'
            else:
                drive(35*turndir,-35*turndir)
        now=time.time()
        if now-last_tx>2:
            wr('d0', f'HELLO botA x={b.x:.2f} y={b.y:.2f}')
            last_tx=now
        msg=rd('d4',0.05)
        if msg: log('RX:', msg)
        if now-last_stat>2:
            st=rd('d6'); d5=rd('d5')
            log(f'pos=({b.x:.2f},{b.y:.2f}) h={b.h:.0f} {st} d5={d5} f={front:.2f} m={mode}')
            last_stat=now
        time.sleep(0.05)
main()
