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

def grad(samples):
    # least squares fit d5 = a*x+b*y+c
    n=len(samples)
    if n<8: return None
    import statistics
    sx=sum(s[0] for s in samples)/n; sy=sum(s[1] for s in samples)/n; sv=sum(s[2] for s in samples)/n
    sxx=syy=sxy=sxv=syv=0
    for x,y,v in samples:
        dx=x-sx; dy=y-sy; dv=v-sv
        sxx+=dx*dx; syy+=dy*dy; sxy+=dx*dy; sxv+=dx*dv; syv+=dy*dv
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-6: return None
    a=(sxv*syy-syv*sxy)/det; b=(syv*sxx-sxv*sxy)/det
    m=math.hypot(a,b)
    if m<1e-4: return None
    return a/m, b/m

def main():
    b=Bot(); last_tx=0; last_stat=0
    samples=[]; lastpos=(0,0); lastmove=time.time()
    mode='seek'; turndir=1; wf_until=0
    best5=0
    log('=== ctrl2 seek start ===')
    while True:
        b.upd()
        l=lidar()
        if l is None:
            drive(0,0); time.sleep(0.2); continue
        l=[x if x>0 else 2.5 for x in l]
        front=min(l[0],l[1],l[15])
        d5=rdf('d5')
        now=time.time()
        if d5 is not None:
            if not samples or (b.x-samples[-1][0])**2+(b.y-samples[-1][1])**2>0.04:
                samples.append((b.x,b.y,d5)); samples=samples[-40:]
            best5=max(best5,d5)
        g=grad(samples)
        # stuck detection
        if (b.x-lastpos[0])**2+(b.y-lastpos[1])**2>0.09:
            lastpos=(b.x,b.y); lastmove=now
        if mode=='seek' and now-lastmove>12:
            mode='wall'; wf_until=now+25; lastmove=now
            log('STUCK -> wall follow')
        if mode=='wall' and now>wf_until:
            mode='seek'; lastmove=now
        if front<0.26:
            left=max(l[3],l[4],l[5]); right=max(l[11],l[12],l[13])
            turndir=1 if left>right else -1
            drive(34*turndir,-34*turndir)
            # commit until front opens
            t0=time.time()
            while time.time()-t0<4:
                l2=lidar()
                if l2 and min(l2[0],l2[1],l2[15])>0.5: break
                time.sleep(0.05)
            drive(0,0)
        elif mode=='wall':
            err=min(l[12],l[13])-0.35
            err=max(-0.25,min(0.25,err))
            turn=int(err*120); sp=55 if front>0.6 else 35
            drive(sp-turn, sp+turn)
        else:
            # score beams
            if g is None: gx,gy=1,0
            else: gx,gy=g
            bestk=0; bestsc=-9
            for k in range(16):
                r=min(l[k], l[(k+1)%16]*1.3+0.2, l[(k-1)%16]*1.3+0.2)
                if r<0.4: continue
                ang=math.radians(b.h+22.5*k)
                al=math.cos(ang)*gx+math.sin(ang)*gy
                sc=al*min(r,1.2)
                if sc>bestsc: bestsc=sc; bestk=k
            # steer toward beam bestk: need heading change of +22.5*bestk (mod)
            dh=(22.5*bestk+180)%360-180
            if abs(dh)>60:
                drive(30 if dh>0 else -30, -30 if dh>0 else 30)
            else:
                turn=int(dh*1.2)
                turn=max(-40,min(40,turn))
                sp=55 if front>0.6 else 35
                drive(sp+turn, sp-turn)
        if now-last_tx>2:
            wr('d0', f'HELLO botA x={b.x:.2f} y={b.y:.2f} d5={d5}')
            last_tx=now
        msg=rd('d4',0.05)
        if msg: log('RX:', msg)
        if now-last_stat>2:
            st=rd('d6')
            log(f'pos=({b.x:.2f},{b.y:.2f}) h={b.h:.0f} {st} d5={d5} best={best5:.2f} f={front:.2f} m={mode} g={g}')
            last_stat=now
        time.sleep(0.05)
main()
