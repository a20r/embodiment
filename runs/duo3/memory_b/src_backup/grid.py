import time, math, json, os
from ctl import Ctl, angdiff

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
    def turn_to(self, target):
        t0=time.time()
        while time.time()-t0<20:
            h=self.heading()
            if h is None: continue
            e=angdiff(target,h)
            if abs(e)<4:
                self.stop(); time.sleep(0.15)
                h=self.heading(); e=angdiff(target,h)
                if abs(e)<6: return True
                continue
            w=max(min(e*2.0,80),-80)
            if abs(w)<12: w=12*(1 if w>0 else -1)
            self.drive(0,w)
            time.sleep(0.1)
        self.stop(); return False

    def fresh_scan(self):
        s=self.scan()
        while s is None: time.sleep(0.05); s=self.scan()
        return [x if x>0 else 0.05 for x in s]
    def stalled(self):
        self.p[7].poll(); return self.p[7].last=='1'

class Explorer:
    def __init__(self):
        self.b=Nav()
        self.cx=0; self.cy=0; self.facing=None
        self.visited=set(); self.walls={}  # (x,y)-> dict dir->open bool
        self.stack=[]
        self.log=open('/memory/grid.log','a',buffering=1)
        self.t0=time.time()
    def lg(self,msg):
        self.log.write(f'{time.time()-self.t0:7.1f} {msg}\n')
    def align(self):
        h=self.b.heading()
        f=round(h/90)%4*90
        self.b.turn_to(f)
        self.facing=f
    def senses(self):
        s=self.b.fresh_scan(); s2=self.b.fresh_scan()
        s=[min(a,b) for a,b in zip(s,s2)]
        return s
    def open_dirs(self,s):
        # returns dict absolute dir->open?, given facing aligned
        res={}
        for k,beam in ((0,0),(90,4),(180,8),(270,12)):
            d=(self.facing+k)%360
            res[d]=s[beam]>0.42
        return res
    def turn_to_dir(self,d):
        if d!=self.facing:
            self.b.turn_to(d)
            self.facing=d
    def advance(self):
        b=self.b
        s=b.fresh_scan(); s0=s[0]
        use_wall = s0<2.8
        target = s0-CELL if use_wall else None
        t_start=time.time(); est=0.0; last=t_start
        stall_ct=0
        ok=True
        while True:
            s=b.fresh_scan()
            now=time.time(); dt=now-last; last=now
            h=b.heading()
            if h is None: h=self.facing
            e=angdiff(self.facing,h)
            w=max(min(1.8*e,35),-35)
            if s[4]<0.45 and s[12]<0.45:
                w+=max(min((s[4]-s[12])*70,25),-25)
            elif s[12]<0.45:
                w+=max(min((0.23-s[12])*70,25),-25)
            elif s[4]<0.45:
                w-=max(min((0.23-s[4])*70,25),-25)
            if s[12]<0.12: w+=20
            if s[4]<0.12: w-=20
            v=0.3 if s[0]>0.55 else 0.15
            if min(s[3],s[4],s[5],s[11],s[12],s[13])<0.12: v=0.12
            if use_wall and s[0]<=target+0.02: break
            if not use_wall and est>=CELL: break
            if s[0]<0.24: break
            if b.stalled():
                stall_ct+=1
                if stall_ct>=2:
                    ok=False; break
            else: stall_ct=0
            if now-t_start>8: ok=False; break
            b.drive(v,w); est+=v*dt
            time.sleep(0.1)
        b.stop(); time.sleep(0.15)
        # forward position correction if wall ahead
        s=b.fresh_scan()
        if s[0]<0.45:
            corr=s[0]-0.30
            if abs(corr)>0.05:
                b.drive(0.12 if corr>0 else -0.12,0)
                time.sleep(abs(corr)/0.12)
                b.stop()
        if not ok:
            self.lg(f'ADV FAIL at ({self.cx},{self.cy}) facing {self.facing} scan={s}')
            # recover: back a bit
            b.drive(-0.15,0); time.sleep(0.8); b.stop()
            return False
        return True
    def checks(self):
        b=self.b
        b.p[9].poll(); b.p[0].poll()
        st=b.p[9].last; s0=b.p[0].last
        if st and 'goal=0' not in st:
            self.lg(f'GOAL FLAG: {st}'); print('GOALFLAG',st)
        if s0 and s0!='0':
            self.lg(f'D0: {s0}')
        for m in b.radio_recv():
            self.lg(f'RX: {m}')
        b.radio_send(f'A cell=({self.cx},{self.cy})')
    def run(self):
        self.align()
        self.lg(f'START h={self.facing}')
        while True:
            self.checks()
            s=self.senses()
            od=self.open_dirs(s)
            key=(self.cx,self.cy)
            self.visited.add(key)
            self.walls[key]={str(d):od[d] for d in od}
            self.lg(f'CELL ({self.cx},{self.cy}) f={self.facing} open={sorted(d for d in od if od[d])} scan0457={[round(s[i],2) for i in (0,4,8,12)]}')
            json.dump({'visited':list(map(list,self.visited)),'walls':{f'{k[0]},{k[1]}':v for k,v in self.walls.items()}},open('/memory/grid.json','w'))
            # choose unvisited open neighbor
            nxt=None
            for d in (self.facing,(self.facing+270)%360,(self.facing+90)%360,(self.facing+180)%360):
                if od.get(d):
                    dx,dy=DIRS[d]
                    if (self.cx+dx,self.cy+dy) not in self.visited:
                        nxt=d; break
            if nxt is None:
                if not self.stack:
                    self.lg('EXPLORATION COMPLETE')
                    print('DONE'); return
                back=self.stack.pop()
                d=back
                self.turn_to_dir(d)
                if self.advance():
                    dx,dy=DIRS[d]; self.cx+=dx; self.cy+=dy
                continue
            self.turn_to_dir(nxt)
            if self.advance():
                dx,dy=DIRS[nxt]; self.cx+=dx; self.cy+=dy
                self.stack.append((nxt+180)%360)
            else:
                # mark as wall to avoid retry loop
                self.walls[key][str(nxt)]=False

if __name__=='__main__':
    Explorer().run()
