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

def analyze():
    pts = scan_pts()
    az=collections.defaultdict(list)
    blob=[]
    for x,y,z in pts:
        r=math.hypot(x,y)
        if r < CAGE: continue
        a=math.degrees(math.atan2(y,x))
        az[int((a+180)/(360/NB))%NB].append(r)
        if r < 0.6: blob.append((a,r,z))
    P={b: sorted(rs)[len(rs)//2] for b,rs in az.items() if len(rs)>=3}
    return P, pts, blob

def main(runtime=520):
    t_end=time.time()+runtime
    log=open('/tmp/ap4.log','a',buffering=1)
    tr=open('/memory/trace.log','a',buffering=1)
    mp=open('/memory/mappts.log','a',buffering=1)
    bl=open('/memory/blob.log','a',buffering=1)
    bot=Bot()
    last=('0','0'); stall=0; mode='explore'; mode_until=0; prev_b=None
    cyc=0
    while time.time()<t_end:
        cyc+=1
        P,pts,blob = analyze()
        cl={}; miss={}
        for b in range(NB):
            w=[P.get((b+o)%NB) for o in range(-4,5)]
            vals=[v for v in w if v is not None]
            cl[b]= min(vals) if vals else 6.0
            miss[b]= sum(1 for v in w if v is None)
        fwd_cl = min(cl[b] for b in range(32,41))
        fwd_miss = sum(miss[b] for b in range(32,41))
        d5 = rl('d5',0.25); d0 = rl('d0',0.2)
        d6d,d9d = bot.sense()
        moved=abs(d6d)+abs(d9d)
        if cyc>4 and moved<8 and last!=('0','0'):
            stall+=1
            wheels(-20,-20,1.0); s=1 if cyc%2 else -1; wheels(14*s,-14*s,1.3)
            log.write(f'{cyc} STALL n={stall}\n'); last=('0','0'); continue
        tgt_az = 0.0; act=''
        if d5=='1' and blob:
            # chase/hold mode: aim at blob centroid azimuth
            # weight azimuths: use circular mean of blob points weighted by (0.6-r)
            sx=sy=0.0
            for a,r,z in blob:
                wgt=(0.6-r)+0.05
                sx+=wgt*math.cos(a*D2R); sy+=wgt*math.sin(a*D2R)
            baz = math.degrees(math.atan2(sy,sx))
            br = sorted(r for a,r,z in blob)[len(blob)//2]
            tgt_az = baz
            if cyc%3==0: wr('d8', f'A POS ({bot.x:.1f},{bot.y:.1f}) d5=1')
            if br > 0.38:  # approach
                v=int(min(22, 60*(br-0.32)))
                diff=max(-14,min(14,int(round(tgt_az*0.7))))
                if diff%2: diff+=1
                wheels(v+diff//2, v-diff//2, 0.6); last=(str(v+diff//2),str(v-diff//2))
                act=f'chase v={v} d={diff}'
            elif br < 0.24:  # too close, back a bit
                wheels(-10,-10,0.5); last=('-10','-10'); act='backoff'
            else:  # hold distance
                stop(); last=('0','0'); act='hold'
                diff=max(-14,min(14,int(round(tgt_az*0.7))))
                if abs(diff)>3:
                    wheels(diff//2,-diff//2,0.4); last=(str(diff//2),str(-diff//2)); act=f'face{diff}'
            mode='chase'
        else:
            mode='explore'
            if time.time()>mode_until:
                mode_until=time.time()+random.uniform(10,20)
            def score(b): return cl[b]+0.35*miss[b]
            best_b = max(range(NB), key=lambda b:(round(score(b),2), -(min(abs(b),NB-abs(b)))))
            if prev_b is not None and score(best_b)-score(prev_b)<0.4:
                best_b=prev_b
            prev_b=best_b
            tgt_az = best_b*5-180
            if fwd_miss>=7:
                err=0.0; v=28; diff=0
                wheels(v,v,0.7); last=(str(v),str(v)); act='void'
            elif abs(tgt_az)>10:
                diff=max(-18,min(18,int(round(tgt_az*0.6))))
                if diff%2: diff+=1
                wheels(diff//2,-diff//2,0.7); last=(str(diff//2),str(-diff//2)); act=f'turn{diff}'
            else:
                v=int(max(8,min(28,55*(min(fwd_cl,1.2)-0.15))))
                diff=max(-10,min(10,int(round(tgt_az*0.5))))
                if diff%2: diff+=1
                wheels(v+diff//2,v-diff//2,0.7); last=(str(v+diff//2),str(v-diff//2)); act=f'fwd{v}'
        if cyc%10==0 and mode=='explore':
            wr('d8', f'A POS ({bot.x:.1f},{bot.y:.1f})')
        phi=bot.H*D2R; c=math.cos(phi); s=math.sin(phi)
        wpts=[]
        for x,y,z in pts:
            r=math.hypot(x,y)
            if r<CAGE: continue
            wpts.append((round(bot.x+x*c-y*s,2), round(bot.y+x*s+y*c,2)))
        mp.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'m':mode,'pts':wpts})+'\n')
        if blob:
            bl.write(json.dumps({'c':cyc,'x':round(bot.x,2),'y':round(bot.y,2),'H':round(bot.H,1),'n':len(blob),'blob':[(round(a),round(r,2)) for a,r,z in blob[:40]]})+'\n')
        fl = rl('d3',0.25)
        log.write(f'{cyc} {act} m={mode} fwd={fwd_cl:.2f} H={bot.H:.1f} xy=({bot.x:.2f},{bot.y:.2f}) d5={d5} d0={d0} {fl}\n')
        tr.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'m':mode,'d5':d5})+'\n')
    stop()
main()
