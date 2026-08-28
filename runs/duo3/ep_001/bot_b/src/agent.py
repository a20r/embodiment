import time, json, math, re, collections, statistics
from grid3 import Nav, Explorer, DIRS
from ctl import angdiff

class Agent(Explorer):
    def __init__(self):
        super().__init__()
        self.mode='explore'
        self.other_goal=False
        self.we_goal=False
        self.svals=collections.deque(maxlen=40)
    def sig(self,dur=1.2):
        t0=time.time(); vals=[]
        while time.time()-t0<dur:
            self.b.p[6].poll()
            if self.b.p[6].queue:
                vals+= [float(x) for x in self.b.p[6].queue]; self.b.p[6].queue.clear()
            time.sleep(0.05)
        return statistics.mean(vals) if vals else None
    def checks(self):
        b=self.b
        b.p[9].poll(); b.p[0].poll(); b.p[6].poll()
        st=b.p[9].last
        if st and 'goal=0' not in st:
            self.lg(f'GOALFLAG {st}'); self.we_goal=True
        for m in b.radio_recv():
            self.lg(f'RX: {m}')
            if re.search('goal.?found|at goal|found.*goal',m,re.I): self.other_goal=True
        s6=b.p[6].last
        b.radio_send(f"A explore {(self.cx,self.cy)} sig={s6}. PLAN: follow me / stay close while I explore. If either finds goal: park+send GOALFOUND, other homes on signal d6")
    def home_step(self):
        """greedy: measure sig, try open dirs, keep best"""
        s=self.b.sc()
        op={(k):v for k,v in zip((0,90,180,270),(s[0],s[4],s[8],s[12]))}
        base=self.sig() or 0
        self.lg(f'HOME base sig={base:.3f} pos={(self.cx,self.cy)}')
        # order dirs by openness
        cand=[d for d in (0,90,180,270) if op[(d-self.facing)%360 if False else d]>0]  # placeholder
        # absolute openness: op keys are relative? no: s[0] is facing dir. map to abs:
        opa={(self.facing+k)%360: v for k,v in zip((0,90,180,270),(s[0],s[4],s[8],s[12]))}
        dirs=[d for d in opa if opa[d]>0.45]
        best=(base,None)
        import random
        random.shuffle(dirs)
        for d in dirs:
            if self.go(d):
                dx,dy=DIRS[d]; self.cx+=dx; self.cy+=dy; self.facing=d
                ns=self.sig() or 0
                self.lg(f'HOME moved {d} sig {base:.3f}->{ns:.3f}')
                if ns>base+0.005:
                    return True
                back=(d+180)%360
                if self.go(back):
                    dx,dy=DIRS[back]; self.cx+=dx; self.cy+=dy; self.facing=back
                else:
                    self.lg('HOME could not return; continue from here')
                    return True
        self.lg('HOME no improving dir; wait')
        time.sleep(3)
        return False
    def run(self):
        h=self.b.heading()
        self.facing=round(h/90)%4*90
        self.lg(f'AGENT start f={self.facing}')
        while True:
            self.checks()
            if self.we_goal:
                self.lg('WE ARE AT GOAL?! parking + beacon')
                self.b.stop()
                while True:
                    self.b.radio_send('A GOALFOUND! home on my signal (s=d6). I am parked at goal.')
                    for m in self.b.radio_recv(): self.lg(f'RX: {m}')
                    time.sleep(2)
            if self.other_goal:
                self.mode='home'
                self.home_step()
                continue
            # explore step (adapted from Explorer.run body)
            ss=[self.b.sc() for _ in range(3)]
            s=[statistics.median(v) for v in zip(*ss)]
            op=self.openness(s)
            od={(self.facing+k)%360:(op[k]>0.45) for k in (0,90,180,270)}
            key=(self.cx,self.cy)
            self.visited.add(key)
            wrec=self.walls.setdefault(key,{})
            for dd in od:
                if wrec.get(str(dd)) is False: od[dd]=False
                else: wrec[str(dd)]=od[dd]
            sixv=self.b.p[6].last
            self.lg(f'CELL {key} f={self.facing} open={sorted(d for d in od if od[d])} sig={sixv}')
            try: json.dump({'visited':sorted(map(list,self.visited)),'walls':{f'{k[0]},{k[1]}':v for k,v in self.walls.items()},'pos':[self.cx,self.cy]},open('/memory/grid.json','w'))
            except Exception: pass
            nxt=None
            for dd in (self.facing,(self.facing+270)%360,(self.facing+90)%360,(self.facing+180)%360):
                if od.get(dd):
                    dx,dy=DIRS[dd]
                    if (self.cx+dx,self.cy+dy) not in self.visited: nxt=dd; break
            if nxt is None:
                if not self.stack:
                    self.lg('COMPLETE reset'); self.visited={key}; continue
                dd=self.stack.pop()
                if self.go(dd):
                    dx,dy=DIRS[dd]; self.cx+=dx; self.cy+=dy; self.facing=dd
                continue
            if self.go(nxt):
                dx,dy=DIRS[nxt]; self.cx+=dx; self.cy+=dy; self.facing=nxt
                self.stack.append((nxt+180)%360)
                time.sleep(0.8)
                sg=self.sig(0.8) or 0
                waits=0
                while sg<0.55 and waits<20:
                    self.b.radio_send(f'A: waiting for you to catch up (sig={sg:.2f}), tail my blob')
                    self.lg(f'WAIT partner sig={sg:.2f}')
                    time.sleep(2); sg=self.sig(1.0) or 0; waits+=1
            else:
                self.walls[key][str(nxt)]=False
                self.fails=getattr(self,'fails',0)+1
                if self.fails>=2:
                    self.fails=0
                    self.wander(15)

    def wander(self,dur):
        self.lg('WANDER')
        b=self.b; t0=time.time()
        while time.time()-t0<dur:
            s=b.sc()
            h=b.heading()
            # steer away from nearest obstacles, prefer forward-open
            best=max(range(16),key=lambda i:s[i])
            e=(best*22.5)
            if e>180: e-=360
            w=max(min(e*0.8,45),-45)
            if s[13]<0.2 or s[14]<0.2 or s[15]<0.2: w+=15
            if s[1]<0.2 or s[2]<0.2 or s[3]<0.2: w-=15
            v=0.3 if s[0]>0.5 else 0.12
            if b.stalled(): v=0.55
            b.drive(v,w)
            time.sleep(0.12)
        b.stop()
        h=self.b.heading()
        self.facing=round(h/90)%4*90
        self.b.spin_to(self.facing)

if __name__=='__main__':
    Agent().run()
