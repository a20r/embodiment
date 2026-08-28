import time, math, json, sys, os
from ctl import Ctl, angdiff

K_V = 0.017
K_W = 0.89

class Nav(Ctl):
    def __init__(self):
        super().__init__()
        self.x=0.0; self.y=0.0
        self.h=None
        self.last_t=time.time()
        self.v_cmd=0.0
    def drive(self, v, w):
        l = -v/K_V - w/(2*K_W)
        r = -v/K_V + w/(2*K_W)
        m=max(abs(l),abs(r))
        if m>60:
            l*=60/m; r*=60/m
        self.wr(4,f'{l:.1f}'); self.wr(5,f'{r:.1f}')
        self.v_cmd=v
    def upd_pose(self):
        t=time.time(); dt=t-self.last_t; self.last_t=t
        hs=self.p[2].last
        if hs:
            try: self.h=float(hs)
            except: pass
        if self.h is not None:
            rad=math.radians(self.h)
            self.x+=self.v_cmd*dt*math.cos(rad)
            self.y+=self.v_cmd*dt*math.sin(rad)

def main():
    b=Nav(); time.sleep(0.3)
    logf=open('/memory/explore.log','a',buffering=1)
    mapf=open('/memory/map.jsonl','a',buffering=1)
    t_start=time.time()
    last_beacon=0; last_log=0
    blocked_turns=0
    while True:
        sc=b.scan()
        b.upd_pose()
        if not sc: continue
        now=time.time()
        eff=[x if x>0 else 0.05 for x in sc]
        front=eff[0]; fl=eff[1]; fr=eff[15]
        right=eff[12]; left=eff[4]; back=eff[8]
        # radio
        for m in b.radio_recv():
            logf.write(f'RX {now-t_start:.1f} {m}\n')
        if now-last_beacon>2:
            b.radio_send(f'HELLO botA pos=({b.x:.1f},{b.y:.1f}) h={b.h}')
            last_beacon=now
        b.p[9].poll(); b.p[0].poll(); b.p[7].poll()
        st=b.p[9].last; s0=b.p[0].last; s7=b.p[7].last
        if (st and 'goal=0' not in st) or (s0 and s0!='0') or (s7 and s7!='0'):
            logf.write(f'ALERT {now-t_start:.1f} d9={st} d0={s0} d7={s7}\n')
        if now-last_log>1.5:
            logf.write(f'T {now-t_start:.1f} pose=({b.x:.2f},{b.y:.2f},{b.h}) f={front:.2f} r={right:.2f} l={left:.2f}\n')
            mapf.write(json.dumps({'t':round(now-t_start,1),'x':round(b.x,2),'y':round(b.y,2),'h':b.h,'scan':sc})+'\n')
            last_log=now
        # escape if touching something
        mn=min(eff); mi=eff.index(mn)
        if mn<0.09:
            # drive away from beam mi
            b.drive(-0.15 if mi<4 or mi>12 else 0.15, 0)
            time.sleep(0.4); b.drive(0,0)
            continue
        # blocked ahead?
        if front<0.28 or (fl<0.15) or (fr<0.15):
            b.drive(0,0)
            # choose: right open? straight impossible; prefer right, then left, then back
            if right>0.6 and blocked_turns<4:
                b.turn_by(-90)
            elif left>0.6 and blocked_turns<4:
                b.turn_by(90)
            else:
                b.turn_by(90)
            blocked_turns+=1
            if blocked_turns>5: blocked_turns=0
            continue
        blocked_turns=0
        # steering: follow right wall at 0.22
        w=0.0
        if right<1.0:
            w = max(min((right-0.22)*-80,35),-35)
            if eff[11]>0 and eff[13]>0 and eff[11]<1.5 and eff[13]<1.5:
                w += max(min((eff[13]-eff[11])*40,25),-25)
        else:
            w=-45  # search right wall
        # avoid left wall too
        if left<0.15: w-=20
        if fl<0.3: w-=25
        if fr<0.3: w+=25
        v=0.3 if front>0.7 else 0.12
        b.drive(v,w)
        time.sleep(0.12)

if __name__=='__main__':
    main()
