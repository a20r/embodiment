import math, time, json, collections, os
from scene import read_line as rl

D2R = math.pi/180
CAGE = 0.14

def wr(port, val):
    for _ in range(3):
        try:
            fd = os.open(f'/dev/robot/{port}', os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd, f'{val}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.02)
def stop(): wr('d1','0'); wr('d7','0')
def wheels(v1, v7, dur, period=0.1):
    end=time.time()+dur
    while time.time()<end:
        wr('d1',str(int(v1))); wr('d7',str(int(v7))); time.sleep(period)
    stop()
def enc(): return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def scan_pts(tries=6):
    for _ in range(tries):
        s = rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

class Bot:
    def __init__(self):
        self.h = heading0 = float(rl('d4',0.5) or 0)
        self.H = 0.0  # unwrapped cumulative heading (deg), world
        self.h0 = self.h
        self.e6, self.e9 = enc()
        self.x=0.0; self.y=0.0
    def sense(self):
        # heading unwrap
        h = float(rl('d4',0.5) or self.h)
        dh = (h - self.h + 180) % 360 - 180
        self.H += dh; self.h = h
        n6, n9 = enc()
        d6 = n6-self.e6; d9 = n9-self.e9
        self.e6, self.e9 = n6, n9
        dC = (d9+d6)/2 * 0.001  # meters (1 unit = 1mm)
        phi = self.H * D2R
        self.x += dC*math.cos(phi); self.y += dC*math.sin(phi)
        return d6, d9

def filtered_prof(nb=72):
    pts = scan_pts()
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y)
        if r < CAGE: continue
        a=math.degrees(math.atan2(y,x))
        az[int((a+180)/(360/nb))%nb].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items() if len(rs)>=3}, len(pts)

def main(runtime=280):
    t_end = time.time()+runtime
    log=open('/tmp/ap2.log','a',buffering=1)
    tr=open('/memory/trace.log','a',buffering=1)
    mp=open('/memory/mappts.log','a',buffering=1)
    bot=Bot()
    last_cmd = ('0','0')
    stall=0
    cyc=0
    while time.time()<t_end:
        cyc+=1
        P,npts = filtered_prof()
        # clearance window: min median over +-4 bins
        cl={}
        for b in range(72):
            w=[P.get((b+o)%72, 6.0) for o in range(-4,5)]
            cl[b]=min(w)
        # candidate dirs: bin's own median must exist (not void-unknown); prefer larger window clearance; void (missing) treated as very open
        def score(b):
            med = P.get(b)
            # void direction: strongly attractive only if truly no data
            base = cl[b]
            return base
        best_b = max(range(72), key=lambda b: (round(score(b),2), -(min(abs(b-0),72-abs(b-0)))))
        tgt_az = best_b*5-180
        fwd_cl = min(cl.get(b,6.0) for b in list(range(0,5))+list(range(67,72)))
        err = ((tgt_az+180)%360)-180
        d6d,d9d = bot.sense()
        moved = abs(d6d)+abs(d9d)
        state='ok'
        if cyc>3 and moved<8 and last_cmd!=('0','0'):
            stall+=1; state='stall'
        if state=='stall':
            wheels(-18,-18,1.0)
            s = 1 if cyc%2 else -1
            wheels(12*s,-12*s,1.2)
            log.write(f'{cyc} STALL n={stall} moved={moved:.0f}\n')
            last_cmd=('0','0'); continue
        if abs(err)>10:
            diff = max(-16,min(16,int(round(err*0.6))))
            if diff%2: diff+=1
            wheels(diff//2, -diff//2, 0.7); last_cmd=(str(diff//2),str(-diff//2))
            act=f'turn{diff}'
        else:
            v = int(max(6, min(24, 50*(fwd_cl-0.15))))
            diff = max(-10,min(10,int(round(err*0.5))))
            if diff%2: diff+=1
            v1=v+diff//2; v7=v-diff//2
            wheels(v1,v7,0.7); last_cmd=(str(v1),str(v7))
            act=f'fwd v={v} diff={diff}'
        # map dump: world coords of scan points
        phi=bot.H*D2R; c=math.cos(phi); s=math.sin(phi)
        pts=scan_pts()
        wpts=[]
        for x,y,z in pts:
            r=math.hypot(x,y)
            if r<CAGE: continue
            wx = bot.x + x*c - y*s; wy = bot.y + x*s + y*c
            wpts.append((round(wx,2),round(wy,2)))
        mp.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'pts':wpts})+'\n')
        log.write(f'{cyc} {act} tgt={tgt_az} fwd={fwd_cl:.2f} best={best_b*5-180}({cl[best_b]:.2f}) H={bot.H:.1f} xy=({bot.x:.2f},{bot.y:.2f}) fl={rl("d3",0.3)} d0={rl("d0",0.2)} d5={rl("d5",0.2)}\n')
        tr.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1)})+'\n')
    stop()

main()
