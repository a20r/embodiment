#!/usr/bin/env python3
"""Reactive left-wall follower with odometry logging.
mode file /bot/mode: 'wall' (follow left wall), 'stop'. Log: /bot/track.log (json lines), pose /bot/pose.txt"""
import rio, ctl, time, math, json, sys
K = 0.00058
side = sys.argv[1] if len(sys.argv) > 1 else "left"   # which wall to follow
import random
SWAP_EVERY = 60.0; _swap_t = [time.time()]
LOG = open("/bot/track.log", "a")
def log(**kw):
    kw["t"] = round(time.time(), 2); LOG.write(json.dumps(kw) + "\n"); LOG.flush()

class Odo:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.h = ctl.heading(5) or 0.0
        self.l, self.r = ctl.enc()
        try:
            p = json.load(open("/bot/pose.txt")); self.x, self.y = p["x"], p["y"]
        except Exception: pass
    def update(self, contact=False):
        l, r = ctl.enc(); h = ctl.heading(1)
        if h is not None:
            d = ctl.angdiff(h, self.h); self.h = (self.h + 0.7*d) % 360
        dl = l - self.l; dr = r - self.r; self.l, self.r = l, r
        if abs(dl) <= 1: dl = 0
        if abs(dr) <= 1: dr = 0
        dist = (dl + dr) / 2.0 * K * (0.5 if contact else 1.0)
        a = math.radians(self.h)
        self.x += dist * math.sin(a); self.y += dist * math.cos(a)

odo = Odo()
class Eff:
    def __init__(self): self.cmd = (0,0); self.t = time.time(); self.enc = ctl.enc(); self.ema = 1.0; self.rest_until = 0
    def set(self, l, r):
        now = time.time(); e = ctl.enc(); dt = now - self.t
        exp = (abs(self.cmd[0]) + abs(self.cmd[1]))/2 * 4.8 * dt
        if exp > 15:
            act = (abs(e[0]-self.enc[0]) + abs(e[1]-self.enc[1]))/2
            self.ema = 0.9*self.ema + 0.1*min(1.5, act/exp)
        self.cmd = (l, r); self.t = now; self.enc = e
        ctl.motors(l, r)
eff = Eff()
_motors = ctl.motors
def b(s, i):
    v = s[i % 16]; return v if v is not None else 2.0

def main():
    last_log = 0; t_start = time.time(); fallback_until = [0]
    while True:
        try: mode = open("/bot/mode").read().strip()
        except: mode = "wall"
        s = ctl.scan(); st = ctl.status(); d5 = rio.read_port(5); d0 = rio.read_port(0); d11 = rio.read_port(11)
        contact = (d5 == "1")
        odo.update(contact)
        rec = dict(x=round(odo.x,3), y=round(odo.y,3), h=round(odo.h,1), scan=s, d0=d0, d5=d5, d11=d11, eff=round(eff.ema,2), **st)
        with open("/bot/pose.txt", "w") as f: f.write(json.dumps(rec))
        if time.time() - last_log > 0.4: log(**rec); last_log = time.time()
        if st.get("goal") == "1":
            ctl.stop(); log(event="GOAL"); open("/bot/mode","w").write("stop"); time.sleep(1); continue

        if time.time() < eff.rest_until:
            ctl.stop(); time.sleep(0.5); continue
        if eff.ema < 0.5:
            eff.rest_until = time.time() + 120; eff.ema = 1.0; log(event="REST", reason="low efficiency"); ctl.stop(); continue
        if mode.startswith("bias"):
            deg = float(mode.split()[1]); tx = odo.x + 1000*math.sin(math.radians(deg)); ty = odo.y + 1000*math.cos(math.radians(deg))
        if mode.startswith("goto") or (mode.startswith("bias") and time.time() > fallback_until[0]):
            if mode.startswith("goto"): _, tx, ty = mode.split(); tx, ty = float(tx), float(ty)
            dx, dy = tx - odo.x, ty - odo.y; dist = math.hypot(dx, dy)
            if dist < 0.15:
                ctl.stop(); log(event="GOTO_REACHED", x=odo.x, y=odo.y); open("/bot/mode","w").write("stop"); continue
            des = math.degrees(math.atan2(dx, dy)) % 360
            # choose best open beam direction near desired
            best = None
            for i in range(16):
                rng = b(s, i); bd = (odo.h + i*22.5) % 360
                if rng < 0.35: continue
                cost = abs(ctl.angdiff(bd, des)) + (60 if rng < 0.6 else 0)
                if best is None or cost < best[0]: best = (cost, bd, rng)
            if mode.startswith("bias") and (best is None or best[0] > 100):
                fallback_until[0] = time.time() + 12; log(event="BIAS_FALLBACK")
            elif best is None:
                ctl.motors(35, -35); time.sleep(0.05); continue
            else:
                e = ctl.angdiff(best[1], odo.h)
                front = min(b(s,0), b(s,1), b(s,15))
                if abs(e) > 40 or front < 0.2:
                    eff.set(30 if e > 0 else -30, -30 if e > 0 else 30); time.sleep(0.03); continue
                corr = max(-20, min(20, e * 0.6))
                # side repulsion
                L = min(b(s,12), b(s,13)*0.92); R = min(b(s,4), b(s,3)*0.92)
                if L < 0.2: corr += (0.2 - L) * 120
                if R < 0.2: corr -= (0.2 - R) * 120
                spd = 34 if front > 0.5 else 26
                eff.set(spd + corr, spd - corr); time.sleep(0.02); continue
        if mode != "wall" and not mode.startswith("bias"):
            ctl.stop(); time.sleep(0.3); continue
        # geometry (mirror for right-wall following)
        global side
        if time.time() - _swap_t[0] > SWAP_EVERY:
            side = "right" if side == "left" else "left"; _swap_t[0] = time.time() + random.uniform(-30, 30); log(event="SWAP", side=side)
        sgn = 1 if side == "left" else -1
        def beam(k): return b(s, -k) if side == "left" else b(s, k)   # k=4 -> wall side beam
        front = min(b(s,0), b(s,1)*1.0, b(s,15)*1.0)
        frontw = min(beam(1), beam(2))          # front-wallside diagonal
        wall = min(beam(4), beam(3)*math.cos(math.radians(22.5)))
        other = min(b(s,4) if side=="left" else b(s,12), 2.0)
        TARGET = 0.24
        if front < 0.22 or frontw < 0.16:
            # blocked ahead: rotate away from wall
            eff.set(sgn*35, -sgn*35)
            time.sleep(0.05); continue
        if wall > 0.5:
            # lost wall: curve toward wall side
            l, r = (20, 34) if side == "left" else (34, 20)
        else:
            err = wall - TARGET          # positive => too far from wall => steer toward wall
            corr = max(-18, min(18, err * 90))
            # also push away if front-diagonal getting close
            if frontw < 0.26: corr -= sgn * 0 + (0.26 - frontw) * 120
            if side == "left": l, r = 32 - corr, 32 + corr
            else: l, r = 32 + corr, 32 - corr
        if contact:  # ease off the wall
            if side == "left": l, r = 30, 15
            else: l, r = 15, 30
        eff.set(l, r)
        time.sleep(0.02)

if __name__ == "__main__":
    while True:
        try: main()
        except Exception:
            import traceback; log(event="CRASH", tb=traceback.format_exc()[-300:]); ctl.stop(); time.sleep(1)
