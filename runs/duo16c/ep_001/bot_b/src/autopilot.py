import math, time, json, collections
from drive import *

D2R = math.pi/180
K_ROT = 0.18       # deg per encoder-unit (right-left)
MM = 0.001         # guess: encoder unit in meters (UNVERIFIED)

class Pose:
    def __init__(self):
        self.h0 = heading(); self.e6, self.e9 = enc()
        self.x = 0.0; self.y = 0.0
    def update(self):
        n6, n9 = enc()
        d6 = n6 - self.e6; d9 = n9 - self.e9
        self.e6, self.e9 = n6, n9
        dR = d9*MM; dL = d6*MM
        dC = (dR+dL)/2
        th = heading()*D2R  # absolute heading (rad), convention?
        # assume d4 is heading CCW-positive; body fwd = (cos th, sin th) in world
        # world frame = initial frame: rotate by -(h0)
        c = math.cos(th - self.h0*D2R); s = math.sin(th - self.h0*D2R)
        self.x += dC*c; self.y += dC*s
        return d6, d9

def clearance_profile(nb=72):
    pts = scan_pts()
    P = prof(pts, nb)
    # clearance for window centered at bin b: min of medians in [b-4..b+4]
    cl = {}
    for b in range(nb):
        cl[b] = min(P.get((b+w)%nb, 9.9) for w in range(-4,5))
    return P, cl

def main():
    log = open('/tmp/ap.log','a', buffering=1)
    trace = open('/memory/trace.log','a', buffering=1)
    pose = Pose()
    stall = 0
    for cyc in range(400):
        P, cl = clearance_profile()
        # target: best clearance window, prefer near forward (bin 0) on ties
        best_b = max(range(72), key=lambda b: (round(cl[b],2), -(min(abs(b), 72-abs(b)))))
        tgt_az = best_b*5 - 180
        fwd_cl = min(cl.get(b,9.9) for b in list(range(-6,7)))
        h = heading()
        err = ((tgt_az + 180) % 360) - 180
        d6d, d9d = pose.update()
        moved = abs(d6d)+abs(d9d)
        state = 'ok'
        # wedge: wheels spun but forward blocked hard
        if fwd_cl < 0.13 and moved > 60 and cyc > 2:
            state = 'wedge'
        if state == 'wedge':
            stall += 1
            wheels('-15','-15', 1.2)
            side = 1 if (cyc % 2) else -1
            wheels(str(10*side), str(-10*side), 1.5)
            log.write(f'{cyc} WEDGE stall={stall} fwd_cl={fwd_cl:.2f} moved={moved}\n')
            continue
        if abs(err) > 8:
            diff = max(-12, min(12, int(err*0.8)))
            if diff % 2: diff += 1
            wheels(str(diff//2), str(-diff//2), 0.8)
            act = f'turn diff={diff}'
        else:
            v = int(max(5, min(22, 60*(fwd_cl-0.12))))
            wheels(str(v), str(v), 0.8)
            act = f'fwd v={v}'
        log.write(f'{cyc} {act} tgt={tgt_az} fwd_cl={fwd_cl:.2f} best_cl={cl[best_b]:.2f} h={h:.1f} xy=({pose.x:.2f},{pose.y:.2f}) d6={pose.e6:.0f} d9={pose.e9:.0f} fl={flags()}\n')
        trace.write(json.dumps({'c':cyc,'x':round(pose.x,3),'y':round(pose.y,3),'h':round(h,1),'fwd':round(fwd_cl,2),'best':best_b,'bc':round(cl[best_b],2)})+'\n')
    stop()
    log.write('DONE\n')

main()
