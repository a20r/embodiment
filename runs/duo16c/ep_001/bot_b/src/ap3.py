import math, time, json, collections, os, random
from scene import read_line as rl

D2R = math.pi/180
CAGE = 0.14
NB = 72

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
def enc(): return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def scan_pts(tries=6):
    for _ in range(tries):
        s = rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

class Bot:
    def __init__(self):
        self.h = float(rl('d4',0.5) or 0)
        self.H = 0.0; self.h0=self.h
        self.e6, self.e9 = enc()
        self.x=0.0; self.y=0.0
    def sense(self):
        h = float(rl('d4',0.5) or self.h)
        self.H += (h - self.h + 180) % 360 - 180; self.h = h
        n6, n9 = enc()
        d6 = n6-self.e6; d9 = n9-self.e9
        self.e6, self.e9 = n6, n9
        dC = (d9+d6)/2 * 0.001
        phi = self.H*D2R
        self.x += dC*math.cos(phi); self.y += dC*math.sin(phi)
        return d6, d9

def fprof():
    pts = scan_pts()
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y)
        if r < CAGE: continue
        a=math.degrees(math.atan2(y,x))
        az[int((a+180)/(360/NB))%NB].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items() if len(rs)>=3}, pts

def main(runtime=500):
    t_end=time.time()+runtime
    log=open('/tmp/ap3.log','a',buffering=1)
    tr=open('/memory/trace.log','a',buffering=1)
    mp=open('/memory/mappts.log','a',buffering=1)
    bot=Bot()
    last=('0','0'); stall=0; mode='wall'; mode_until=0
    cyc=0
    while time.time()<t_end:
        cyc+=1
        P,pts = fprof()
        cl={}; miss={}
        for b in range(NB):
            w=[P.get((b+o)%NB) for o in range(-4,5)]
            vals=[v for v in w if v is not None]
            cl[b]= min(vals) if vals else 6.0
            miss[b]= sum(1 for v in w if v is None)
        fwd_cl = min(cl[b] for b in range(32,41))
        fwd_miss = sum(miss[b] for b in range(32,41))
        def score(b): return cl[b] + 0.35*miss[b]
        if time.time()>mode_until:
            mode = random.choice(['wall','explore','explore'])
            mode_until = time.time()+random.uniform(12,25)
        if mode=='explore':
            best_b = max(range(NB), key=lambda b:(round(score(b),2), -(min(abs(b),NB-abs(b)))))
        else:
            best_b = max(range(NB), key=lambda b:(round(cl[b],2), -(min(abs(b),NB-abs(b)))))
        # hysteresis: keep prev target unless clearly better
        if 'prev_b' in dir(bot) and bot.prev_b is not None:
            if score(best_b) - score(bot.prev_b) < 0.4:
                best_b = bot.prev_b
        bot.prev_b = best_b
        tgt_az = best_b*5-180
        err = ((tgt_az+180)%360)-180
        d6d,d9d = bot.sense()
        moved=abs(d6d)+abs(d9d)
        if cyc>4 and moved<8 and last!=('0','0'):
            stall+=1
            wheels(-20,-20,1.0); s=1 if cyc%2 else -1; wheels(14*s,-14*s,1.3)
            log.write(f'{cyc} STALL n={stall}\n'); last=('0','0'); continue
        if fwd_miss >= 7:
            err = 0
            act = 'void'
        if abs(err)>10:
            diff=max(-18,min(18,int(round(err*0.6))))
            if diff%2: diff+=1
            wheels(diff//2,-diff//2,0.7); last=(str(diff//2),str(-diff//2)); act=f'turn{diff}'
        else:
            v=int(max(8,min(30,55*(min(fwd_cl,1.2)-0.15))))
            diff=max(-10,min(10,int(round(err*0.5))))
            if diff%2: diff+=1
            wheels(v+diff//2, v-diff//2, 0.7); last=(str(v+diff//2),str(v-diff//2)); act=f'fwd{v}'
        if cyc%8==0:
            wr('d8', f'PING A ({bot.x:.1f},{bot.y:.1f})')
        phi=bot.H*D2R; c=math.cos(phi); s=math.sin(phi)
        wpts=[]
        for x,y,z in pts:
            r=math.hypot(x,y)
            if r<CAGE: continue
            wpts.append((round(bot.x+x*c-y*s,2), round(bot.y+x*s+y*c,2)))
        mp.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'m':mode,'pts':wpts})+'\n')
        fl = rl('d3',0.3); d0 = rl('d0',0.2); d5=rl('d5',0.2)
        log.write(f'{cyc} {act} m={mode} tgt={tgt_az} fwd={fwd_cl:.2f} H={bot.H:.1f} xy=({bot.x:.2f},{bot.y:.2f}) {fl} d0={d0} d5={d5}\n')
        tr.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'m':mode})+'\n')
    stop()
main()
