import math, time, json, collections, os, random
from scene import read_line as rl

D2R=math.pi/180; CAGE=0.14; NB=72
def wr(p,v):
    for att in range(4):
        try:
            fl=os.O_WRONLY|os.O_NONBLOCK if att<2 else os.O_RDWR|os.O_NONBLOCK
            fd=os.open(f'/dev/robot/{p}',fl)
            os.write(fd,f'{int(v)}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.02)
def stop(): wr('d1',0); wr('d7',0)
def drive(a,b,dur,period=0.08):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(period)
def enc(): return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def scan_pts(tries=6):
    for _ in range(tries):
        s=rl('d2',1.0)
        if s: return [tuple(map(float,t.split(','))) for t in s.split(';') if t]
        time.sleep(0.05)
    return []
def rpts_of(pts):
    out=[]
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.degrees(math.atan2(y,x))
        if r<0.12: continue                      # inner cage
        if r<0.55 and not (-50<=a<=35): continue # own body ring (robot-frame)
        out.append((r,a))
    return out
def clr(rp,lo,hi):
    rs=[r for r,a in rp if r>=CAGE and lo<=a<=hi]
    return min(rs) if rs else 3.0

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
    def turn_to(self,tgt,tol=8,max_s=6):
        t0=time.time()
        while time.time()-t0<max_s:
            err=(tgt-self.H+180)%360-180
            if abs(err)<=tol: stop(); return True
            diff=max(-16,min(16,int(round(err*0.5))))
            if diff%2: diff+=1
            drive(diff//2,-diff//2,0.3)
            self.sense()
        stop(); return False

def main(runtime=560):
    t_end=time.time()+runtime
    log=open('/tmp/ap6.log','a',buffering=1)
    tr=open('/memory/trace.log','a',buffering=1)
    mp=open('/memory/mappts.log','a',buffering=1)
    bot=Bot()
    last=('0','0'); phase='leg'; sweep_dir=1; side_left=0.0
    cyc=0; last_ping=0
    while time.time()<t_end:
        cyc+=1
        pts=scan_pts(); rp=rpts_of(pts)
        fwd=clr(rp,-15,15); back=clr(rp,165,195)
        L=clr(rp,10,50); R=clr(rp,-50,-10)
        minall=min(clr(rp,a,a+30) for a in range(-180,180,30))
        d6d,d9d=bot.sense(); moved=abs(d6d)+abs(d9d)
        d5=rl('d5',0.25); d0=rl('d0',0.2); fl=rl('d3',0.25)
        if d0 not in ('0','') or 'goal=1' in fl or 'here=1' in fl:
            log.write(f'!!! EVENT c={cyc} d0={d0} fl={fl} xy=({bot.x:.2f},{bot.y:.2f})\n')
            open('/memory/EVENT.log','a').write(json.dumps({'t':time.time(),'c':cyc,'d0':d0,'fl':fl,'x':bot.x,'y':bot.y,'H':bot.H,'pts':[(round(x,2),round(y,2),round(z,2)) for x,y,z in pts]})+'\n')
        act=''
        blob=[(x,y) for x,y,z in pts if CAGE<=math.hypot(x,y)<0.55]
        if d5=='1' and len(blob)>30:
            sx=sy=0.0
            for x,y in blob:
                r=math.hypot(x,y); wgt=(0.55-r)+0.05; sx+=wgt*x; sy+=wgt*y
            baz=math.degrees(math.atan2(sy,sx))
            br=sorted(math.hypot(x,y) for x,y in blob)[len(blob)//2]
            if cyc%4==0: wr('d8',f'A({bot.x:.0f},{bot.y:.0f})')
            err=(baz+180)%360-180
            if br>0.42 and fwd>0.4:
                if abs(err)>30: bot.turn_to(bot.H+err)
                else:
                    diff=max(-12,min(12,int(round(err*0.6))))
                    if diff%2: diff+=1
                    v=int(min(18,50*(br-0.35)))
                    drive(v+diff//2,v-diff//2,0.6); last=(str(v+diff//2),str(v-diff//2))
                act=f'chase az={baz:.0f} r={br:.2f}'
            elif br<0.22:
                drive(-10,-10,0.4); last=('-10','-10'); act='backoff'
            else:
                stop(); last=('0','0'); act='hold'
                if cyc%2: wr('d8',f'A-NEAR({bot.x:.0f},{bot.y:.0f})')
        elif minall<0.42:
            # tight: arc escape, NO pivot
            if fwd>max(back,0.4):
                diff=10 if L>R else -10
                drive(16+diff//2,16-diff//2,0.8); last=('x','x'); act=f'escF{diff}'
            elif back>0.4:
                diff=10 if L>R else -10
                drive(-14-diff//2,-14+diff//2,0.8); last=('x','x'); act=f'escB{diff}'
            else:
                drive(-12,-12,0.5); drive(-8,8,0.5); last=('x','x'); act='escWig'
        else:
            if fwd<0.55 or fwd>4.5:
                sweep_dir*=-1
                bot.turn_to(bot.H+90*sweep_dir)
                side_left=1.3; phase='side'; act='t90'
            else:
                if phase=='leg':
                    v=int(max(10,min(26,50*(fwd-0.25))))
                    drive(v,v,0.7); last=(str(v),str(v)); act=f'leg{v}'
                else:
                    side_left-=moved*0.001
                    if side_left<=0 or fwd<0.45:
                        bot.turn_to(bot.H+90*sweep_dir); phase='leg'; act='t90b'
                    else:
                        v=int(max(8,min(16,40*(fwd-0.2))))
                        drive(v,v,0.7); last=(str(v),str(v)); act=f'side{v}'
        if time.time()-last_ping>15:
            wr('d8',f'A({bot.x:.0f},{bot.y:.0f})'); last_ping=time.time()
        phi=bot.H*D2R; c=math.cos(phi); s=math.sin(phi)
        wpts=[(round(bot.x+x*c-y*s,2),round(bot.y+x*s+y*c,2)) for x,y,z in pts if math.hypot(x,y)>=CAGE]
        mp.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'pts':wpts})+'\n')
        log.write(f'{cyc} {act} fwd={fwd:.2f} minall={minall:.2f} H={bot.H:.1f} xy=({bot.x:.2f},{bot.y:.2f}) d5={d5} d0={d0} {fl}\n')
        tr.write(json.dumps({'c':cyc,'x':round(bot.x,3),'y':round(bot.y,3),'H':round(bot.H,1),'d5':d5})+'\n')
    stop()

if __name__=='__main__':
    main()
