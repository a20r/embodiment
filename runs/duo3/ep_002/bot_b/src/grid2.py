import time, math, json, os
from ctl import Ctl, angdiff
from unstick import unstick, spin_to, sc

K_V=0.017; K_W=0.89
CELL=0.6
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}

class Nav(Ctl):
    def drive(self,v,w):
        l=-v/K_V - w/(2*K_W); r=-v/K_V + w/(2*K_W)
        m=max(abs(l),abs(r))
        if m>60: l*=60/m; r*=60/m
        self.wr(4,f'{l:.1f}'); self.wr(5,f'{r:.1f}')
    def stop(self): self.drive(0,0)
    def stalled(self):
        self.p[7].poll(); return self.p[7].last=='1'

class Explorer:
    def __init__(self):
        self.b=Nav()
        self.cx=0; self.cy=0; self.facing=0
        self.visited=set(); self.walls={}
        self.stack=[]
        self.log=open('/memory/grid.log','a',buffering=1)
        self.t0=time.time()
    def lg(self,m): self.log.write(f'{time.time()-self.t0:7.1f} {m}\n')
    def scan2(self):
        s=sc(self.b); time.sleep(0.05); s2=sc(self.b)
        return [min(a,b) for a,b in zip(s,s2)]
    def openness(self,s):
        # relative: front,left,back,right min-triples
        def trip(c): return min(s[(c-1)%16],s[c],s[(c+1)%16])
        return {0:trip(0),90:trip(4),180:trip(8),270:trip(12)}
    def face(self,d):
        if d!=self.facing or True:
            spin_to(self.b,d)
        self.facing=d
    def checks(self):
        b=self.b
        b.p[9].poll(); b.p[0].poll()
        st=b.p[9].last; s0=b.p[0].last
        if st and 'goal=0' not in st:
            self.lg(f'GOALFLAG {st}')
        if s0 and s0 not in ('0',None):
            self.lg(f'D0 {s0}')
        for m in b.radio_recv():
            self.lg(f'RX: {m}')
        b.radio_send(f'A cell=({self.cx},{self.cy}) t={int(time.time()-self.t0)}')
    def advance(self):
        b=self.b
        s=self.scan2(); s0=s[0]
        use_wall = s0<2.7
        target=s0-CELL
        est=0.0; last=time.time(); t_start=last
        stall_t=None
        while True:
            s=sc(b)
            now=time.time(); dt=now-last; last=now
            h=b.heading()
            e=angdiff(self.facing,h) if h is not None else 0
            w=max(min(1.8*e,35),-35)
            if s[4]<0.45 and s[12]<0.45:
                w+=max(min((s[4]-s[12])*70,25),-25)
            elif s[12]<0.4:
                w+=max(min((0.23-s[12])*70,20),-20)
            elif s[4]<0.4:
                w-=max(min((0.23-s[4])*70,20),-20)
            v=0.28 if s[0]>0.6 else 0.15
            if use_wall and s[0]<=target+0.02: break
            if not use_wall and est>=CELL: break
            if s[0]<0.22: break
            if b.stalled():
                if stall_t is None: stall_t=now
                elif now-stall_t>0.5:
                    self.lg(f'STALL during adv, unstick')
                    b.stop()
                    unstick(b)
                    spin_to(b,self.facing)
                    stall_t=None
                    ns=sc(b)
                    if use_wall: target=min(target, ns[0]-0.15) if False else target
                    last=time.time()
                    continue
            else: stall_t=None
            if now-t_start>10:
                b.stop(); return False
            b.drive(v,w); est+=v*dt
            time.sleep(0.1)
        b.stop(); time.sleep(0.1)
        s=sc(b)
        if s[0]<0.5:
            corr=s[0]-0.30
            if abs(corr)>0.05:
                b.drive(0.12 if corr>0 else -0.12,0)
                time.sleep(min(abs(corr)/0.12,1.5)); b.stop()
        return True
    def run(self):
        h=self.b.heading()
        self.face(round(h/90)%4*90)
        self.lg(f'START2 facing={self.facing}')
        while True:
            self.checks()
            if min(sc(self.b))<0.12:
                unstick(self.b); spin_to(self.b,self.facing)
            s=self.scan2()
            op=self.openness(s)
            od={(self.facing+k)%360: (op[k]>0.45) for k in (0,90,180,270)}
            key=(self.cx,self.cy)
            self.visited.add(key)
            self.walls.setdefault(key,{}).update({str(d):od[d] for d in od})
            self.lg(f'CELL ({self.cx},{self.cy}) f={self.facing} open={sorted(d for d in od if od[d])} op={ {k:round(v,2) for k,v in op.items()} }')
            try:
                json.dump({'visited':list(map(list,self.visited)),'walls':{f'{k[0]},{k[1]}':v for k,v in self.walls.items()},'pos':[self.cx,self.cy]},open('/memory/grid.json','w'))
            except Exception: pass
            nxt=None
            for d in (self.facing,(self.facing+270)%360,(self.facing+90)%360):
                if od.get(d):
                    dx,dy=DIRS[d]
                    if (self.cx+dx,self.cy+dy) not in self.visited:
                        nxt=d; break
            if nxt is None:
                if not self.stack:
                    self.lg('COMPLETE'); return
                d=self.stack.pop()
                self.face(d)
                if self.advance():
                    dx,dy=DIRS[d]; self.cx+=dx; self.cy+=dy
                else:
                    self.lg('BACKTRACK ADV FAIL')
                    unstick(self.b)
                continue
            self.face(nxt)
            if self.advance():
                dx,dy=DIRS[nxt]; self.cx+=dx; self.cy+=dy
                self.stack.append((nxt+180)%360)
            else:
                self.lg(f'ADV FAIL dir {nxt}')
                self.walls[key][str(nxt)]=False
                unstick(self.b)

if __name__=='__main__':
    Explorer().run()
