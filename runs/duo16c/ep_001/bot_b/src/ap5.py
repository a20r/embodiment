import math, time, json, collections, os, random
from scene import read_line as rl

D2R=math.pi/180; CAGE=0.14; NB=72
def wr(p,v):
    for attempt in range(4):
        try:
            flags = os.O_WRONLY|os.O_NONBLOCK if attempt<2 else os.O_RDWR|os.O_NONBLOCK
            fd=os.open(f'/dev/robot/{p}',flags)
            os.write(fd,f'{int(v)}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.02)
def stop(): wr('d1',0); wr('d7',0)
def wheels(a,b,dur,period=0.1):
    end=time.time()+dur
    while time.time()<end:
        wr('d1',a); wr('d7',b); time.sleep(period)
def enc(): return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def scan_pts(tries=6):
    for _ in range(tries):
        s=rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []

class Bot:
    def __init__(self):
        self.h=float(rl('d4',0.5) or 0); self.H=0.0; self.h0=self.h
        self.e6,self.e9=enc(); self.x=0.0; self.y=0.0
    def sense(self):
        h=float(rl('d4',0.5) or self.h)
        self.H+=(h-self.h+180)%360-180; self.h=h
        n6,n9=enc(); d6=n6-self.e6; d9=n9-self.e9; self.e6,self.e9=n6,n9
        dC=(d9+d6)/2*0.001; phi=self.H*D2R
        self.x+=dC*math.cos(phi); self.y+=dC*math.sin(phi)
        return d6,d9
    def turn_to(self, tgt, tol=7, max_s=7):
        t0=time.time()
        while time.time()-t0<max_s:
            err=(tgt-self.H+180)%360-180
            if abs(err)<=tol: stop(); return True
            diff=max(-20,min(20,int(round(err*0.5))))
            if diff%2: diff+=1
            wheels(diff//2,-diff//2,0.25)
            self.sense()
        stop(); return False

def clusters(pts):
    cell=0.12
    g=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y)
        if r<CAGE: continue
        g[(int(x/cell),int(y/cell))].append((x,y,r))
    seen=set(); out=[]
    for k in list(g):
        if k in seen: continue
        stack=[k]; comp=[]
        while stack:
            c=stack.pop()
            if c in seen: continue
            seen.add(c); comp.extend(g[c])
            for dx in(-1,0,1):
                for dy in(-1,0,1):
                    n=(c[0]+dx,c[1]+dy)
                    if n in g and n not in seen: stack.append(n)
        if comp:
            xs=[p[0] for p in comp]; ys=[p[1] for p in comp]
            cx=sum(xs)/len(xs); cy=sum(ys)/len(ys)
            ext=max(max(xs)-min(xs),max(ys)-min(ys))
            out.append({'n':len(comp),'r':round(math.hypot(cx,cy),2),'az':round(math.degrees(math.atan2(cy,cx))),'ext':round(ext,2)})
    return out

def main(runtime=560):
    t_end=time.time()+runtime
    log=open('/tmp/ap5.log','a',buffering=1)
    tr=open('/memory/trace.log','a',buffering=1)
    mp=open('/memory/mappts.log','a',buffering=1)
    cl_log=open('/memory/clusters.log','a',buffering=1)
    bot=Bot()
    last=('0','0'); sweep_dir=1; phase='leg'; side_left=0.0
    cyc=0; last_ping=0
    while time.time()<t_end:
        cyc+=1
        pts=scan_pts()
        az=collections.defaultdict(list)
        for x,y,z in pts:
            r=math.hypot(x,y)
            if r<CAGE: continue
            az[int((math.degrees(math.atan2(y,x))+180)/(360/NB))%NB].append(r)
        P={b:sorted(rs)[len(rs)//2] for b,rs in az.items() if len(rs)>=3}
        cl={}; miss={}
        for b in range(NB):
            w=[P.get((b+o)%NB) for o in range(-4,5)]
            vals=[v for v in w if v is not None]
            cl[b]=min(vals) if vals else 6.0
            miss[b]=sum(1 for v in w if v is None)
        fwd_cl=min(cl[b] for b in range(32,41))
        d6d,d9d=bot.sense()
        moved=abs(d6d)+abs(d9d)
        d5=rl('d5',0.25); d0=rl('d0',0.2); fl=rl('d3',0.25)
        if d0 not in ('0','') or 'goal=1' in fl or 'here=1' in fl:
            log.write(f'!!! EVENT c={cyc} d0={d0} fl={fl}\n')
            open('/memory/EVENT.log','a').write(json.dumps({'t':time.time(),'c':cyc,'d0':d0,'fl':fl,'x':bot.x,'y':bot.y,'H':bot.H,'pts':[(round(x,2),round(y,2),round(z,2)) for x,y,z in pts]})+'\n')
        cls=clusters(pts)
        cl_log.write(json.dumps({'c':cyc,'x':round(bot.x,2),'y':round(bot.y,2),'H':round(bot.H,1),'cls':cls})+'\n')
        if cyc%15==0:
            # control sanity: command 8,8 for 0.5s, expect encoder motion
            wheels(8,8,0.5); bot.sense()
            chk=abs(bot.e6-bot.e6)+0
        if cyc>4 and moved<8 and last!=('0','0'):
            wheels(-20,-20,1.0); wheels(16,-16,1.0); bot.sense()
            log.write(f'{cyc} STALL\n'); last=('0','0'); continue
        act=''
        # slip/wedge detection: encoders running hot while scene static & wall close
        global wedge_n
        slip = moved>250
        if phase=='leg' and fwd_cl<0.6 and (slip or moved<5):
            wedge_n = getattr(bot,'wedge_n',0)+1
        else:
            bot.wedge_n=0
        if getattr(bot,'wedge_n',0)>=3:
            bot.wedge_n=0
            wheels(-26,-26,1.6); sgn=1 if cyc%2 else -1
            bot.turn_to(bot.H+sgn*random.uniform(70,140))
            phase='leg'
            log.write(f'{cyc} WEDGE-RECOVERY fwd={fwd_cl:.2f} moved={moved:.0f}\n')
            last=('0','0'); continue
        blob=[(x,y) for x,y,z in pts if CAGE<=math.hypot(x,y)<0.55]
        if d5=='1' and blob:
            sx=sy=0.0
            for x,y in blob:
                r=math.hypot(x,y); wgt=(0.55-r)+0.05
                sx+=wgt*x; sy+=wgt*y
            baz=math.degrees(math.atan2(sy,sx))
            br=sorted(math.hypot(x,y) for x,y in blob)[len(blob)//2]
            if cyc%4==0: wr('d8',f'A({bot.x:.0f},{bot.y:.0f})')
            err=(baz+180)%360-180
            if br>0.40:
                if abs(err)>25: bot.turn_to(bot.H+err)
                else:
                    diff=max(-12,min(12,int(round(err*0.6))))
                    if diff%2: diff+=1
                    v=int(min(20,55*(br-0.34)))
                    wheels(v+diff//2,v-diff//2,0.5); last=(str(v+diff//2),str(v-diff//2))
                act=f'chase az={baz:.0f} r={br:.2f}'
            elif br<0.22:
                wheels(-10,-10,0.4); last=('-10','-10'); act='backoff'
            else:
                stop(); last=('0','0'); act='hold'
                if cyc%2: wr('d8',f'A-NEAR({bot.x:.0f},{bot.y:.0f})')
        else:
            if phase=='leg':
                if fwd_cl<0.55:
                    sweep_dir*=-1
                    bot.turn_to(bot.H+90*sweep_dir)
                    side_left=1.3; phase='side'; act='leg->side'
                else:
                    v=int(max(10,min(26,50*(fwd_cl-0.25))))
                    wheels(v,v,0.6); last=(str(v),str(v)); act=f'leg{v}'
            else:
                side_left-=moved*0.001
                if side_left<=0 or fwd_cl<0.4:
                    bot.turn_to(bot.H+90*sweep_dir)
                    phase='leg'; act='side->leg'
                else:
                    v=int(max(8,min(18,40*(fwd_cl-0.2))))
                    wheels(v,v,0.6); last=(str(v),str(v)); act=f'side{v}'
        if time.time()-last_ping>14:
            wr('d8',f'A({bot.x:.0f},{bot.y:.0f})'); last_ping=time.time()
        phi=bot.H*D2R; c=math.cos(phi); s=math.sin(phi)
        wpts=[(round(bot.x+x*c-y*s,2),round(bot.y+x*s+y*c,2)) for x,y,z in pts if math.hypot(x,y)>=CAGE]
        mp.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'pts':wpts})+'\n')
        log.write(f'{cyc} {act} fwd={fwd_cl:.2f} H={bot.H:.1f} xy=({bot.x:.2f},{bot.y:.2f}) d5={d5} d0={d0} {fl}\n')
        tr.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'d5':d5})+'\n')
    stop()

if __name__=='__main__':
    main()
