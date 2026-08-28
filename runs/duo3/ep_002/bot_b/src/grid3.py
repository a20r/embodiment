import time, math, json, os
from ctl import Ctl, angdiff

K_W=0.89
CELL=0.57
DIRS={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}

def cmd_for(v):  # v m/s -> wheel cmd
    c=v/0.0066
    return c

class Nav(Ctl):
    def wheels(self,l,r):
        self.wr(4,f'{l:.1f}'); self.wr(5,f'{r:.1f}')
    def drive(self,v,w):  # v m/s fwd, w deg/s
        base=cmd_for(v)
        dl=-w/(2*K_W); dr=w/(2*K_W)
        self.wheels(base+dl, base+dr)
    def stop(self): self.wheels(0,0)
    def stalled(self):
        self.p[7].poll(); return self.p[7].last=='1'
    def sc(self):
        s=self.scan()
        while s is None:
            time.sleep(0.05); s=self.scan()
        return [x if x>0 else 0.05 for x in s]
    def spin_to(self,target,tol=5):
        t0=time.time()
        while time.time()-t0<14:
            h=self.heading()
            if h is None: continue
            e=angdiff(target,h)
            if abs(e)<tol:
                self.stop(); return True
            w=max(min(e*1.8,80),-80)
            if abs(w)<25: w=25*(1 if w>0 else -1)
            self.wheels(-w/1.78, w/1.78)
            time.sleep(0.1)
        self.stop(); return False

class Explorer:
    def __init__(self):
        self.b=Nav()
        self.cx=0; self.cy=0; self.facing=0
        self.visited=set(); self.stack=[]
        self.walls={}
        self.log=open('/memory/grid.log','a',buffering=1)
        self.t0=time.time()
        self.rxlog=[]
    def lg(self,m): self.log.write(f'{time.time()-self.t0:7.1f} {m}\n')
    def checks(self):
        b=self.b
        b.p[9].poll(); b.p[0].poll()
        st=b.p[9].last; s0=b.p[0].last
        if st and 'goal=0' not in st: self.lg(f'GOALFLAG {st}')
        if s0 and s0!='0': self.lg(f'D0 {s0}')
        for m in b.radio_recv():
            self.lg(f'RX: {m}')
        b.radio_send(f'A cell=({self.cx},{self.cy})')
    def openness(self,s):
        return {0:s[0],90:s[4],180:s[8],270:s[12]}
    def go(self,d):
        """move one cell toward absolute dir d. return True if advanced."""
        b=self.b
        b.spin_to(d)
        self.facing=d
        s=b.sc(); s0=s[0]
        use_wall=s0<2.7
        target=s0-CELL
        t0=time.time(); last=t0; est=0.0
        stall_t=None; power=False
        arrived=False
        while True:
            s=b.sc()
            b.p[9].poll()
            if b.p[9].last and 'goal=0' not in b.p[9].last: self.lg(f'GOALFLAG-MOVE {b.p[9].last}')
            now=time.time(); dt=now-last; last=now
            est+=(0.4 if power else 0.28)*dt
            h=b.heading(); e=angdiff(d,h) if h is not None else 0
            w=max(min(1.6*e,30),-30)
            if s[4]<0.45 and s[12]<0.45:
                w+=max(min((s[4]-s[12])*60,20),-20)
            elif s[12]<0.35: w+=max(min((0.22-s[12])*80,20),0)
            elif s[4]<0.35: w-=max(min((0.22-s[4])*80,20),0)
            if use_wall and s[0]<=target+0.03: arrived=True; break
            if s[0]<0.24:
                arrived = est>=0.38; break
            if not use_wall and est>=CELL: arrived=True; break
            if now-t0>12: b.stop(); return False
            if b.stalled():
                if stall_t is None: stall_t=now
                elif now-stall_t>0.6 and not power:
                    power=True; stall_t=None
                elif stall_t and now-stall_t>1.2 and power:
                    b.stop(); self.lg('GO stall hard'); return False
            else:
                stall_t=None; power=False
            v=0.55 if power else (0.33 if s[0]>0.6 else 0.2)
            b.drive(v,w)
            time.sleep(0.08)
        b.stop(); time.sleep(0.12)
        if not arrived: return False
        s=b.sc()
        if s[0]<0.5:
            corr=s[0]-0.28
            if corr<-0.04:
                b.drive(-0.2,0); time.sleep(min(-corr/0.2,1.0)); b.stop()
            elif corr>0.04 and corr<0.3:
                b.drive(0.2,0); time.sleep(min(corr/0.2,1.0)); b.stop()
        return True
    def run(self):
        h=self.b.heading()
        self.facing=round(h/90)%4*90
        self.lg(f'START3 f={self.facing}')
        fails=0
        while True:
            self.checks()
            import statistics
            ss=[self.b.sc() for _ in range(3)]
            s=[statistics.median(v) for v in zip(*ss)]
            op=self.openness(s)
            od={(self.facing+k)%360:(op[k]>0.45) for k in (0,90,180,270)}
            key=(self.cx,self.cy)
            self.visited.add(key)
            self.walls.setdefault(key,{}).update({str(dd):od[dd] for dd in od})
            self.lg(f'CELL {key} f={self.facing} open={sorted(d for d in od if od[d])} op={ {k:round(v,2) for k,v in op.items()} }')
            try: json.dump({'visited':sorted(map(list,self.visited)),'walls':{f'{k[0]},{k[1]}':v for k,v in self.walls.items()},'pos':[self.cx,self.cy]},open('/memory/grid.json','w'))
            except Exception: pass
            nxt=None
            for dd in (self.facing,(self.facing+270)%360,(self.facing+90)%360,(self.facing+180)%360):
                if od.get(dd):
                    dx,dy=DIRS[dd]
                    if (self.cx+dx,self.cy+dy) not in self.visited: nxt=dd; break
            if nxt is None:
                if not self.stack:
                    self.lg('COMPLETE - reset visited, continue')
                    self.visited={(self.cx,self.cy)}
                    continue
                dd=self.stack.pop()
                if self.go(dd):
                    dx,dy=DIRS[dd]; self.cx+=dx; self.cy+=dy; self.facing=dd
                else:
                    self.lg('BT fail')
                continue
            if self.go(nxt):
                dx,dy=DIRS[nxt]; self.cx+=dx; self.cy+=dy; self.facing=nxt
                self.stack.append((nxt+180)%360)
                fails=0
            else:
                self.walls[key][str(nxt)]=False
                fails+=1
                if fails>3:
                    # wiggle
                    self.b.drive(-0.3,0); time.sleep(0.6); self.b.stop(); fails=0

if __name__=='__main__':
    Explorer().run()
