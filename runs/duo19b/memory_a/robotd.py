#!/usr/bin/env python3
"""Robot daemon v2. Commands appended to /tmp/cmd:
 wheels L R | stop | rot H | hop H DIST | auto N | tx MSG | setpos X Y | quit
State: /tmp/state.json  Log: /tmp/log.jsonl  Radio: /tmp/rx.log  Events: /tmp/events.log
Frame: x=cos(h), y=sin(h) (internal; may be mirror of true geography). Beam i at angle h+22.5*i."""
import rb, time, json, math, os, threading, statistics
TICKS_PER_M = 3000.0
CMD = "/tmp/cmd"; STATE = "/tmp/state.json"; LOG = "/tmp/log.jsonl"; RXLOG = "/tmp/rx.log"; EV = "/tmp/events.log"
st = dict(x=0.0, y=0.0, h=0.0, hraw=0.0, encL=None, encR=None, scan=None, d0=0, d5=0, d11=0.0, tick=0, goal=0, here=0, t=0.0, wheels=(0,0), busy="")
lock = threading.Lock(); hbuf = []
visited = set(); junctions = []; goal_pos = None; target = None; samples = []
def ev(msg):
    with open(EV, "a") as f: f.write("%.1f %s\n" % (time.time(), msg))
def set_wheels(l, r):
    rb.write_line(1, str(int(l))); rb.write_line(7, str(int(r))); st["wheels"] = (l, r)
def angdiff(a, b): return (a - b + 180) % 360 - 180
def cmean(vals):
    x = sum(math.cos(math.radians(v)) for v in vals); y = sum(math.sin(math.radians(v)) for v in vals)
    return math.degrees(math.atan2(y, x)) % 360
def get(k):
    with lock: return st[k]
def cell(x, y): return (int(math.floor(x / 0.5)), int(math.floor(y / 0.5)))

def sensor_loop():
    global goal_pos
    lastE = None
    while True:
        try:
            h = rb.heading()
            if h is not None: hbuf.append(h); del hbuf[:-4]
            eL = rb.d(9); eR = rb.d(6); s = rb.scan(); d0 = rb.d(0); d5 = rb.d(5); d11 = rb.d(11); stt = rb.status()
            with lock:
                if hbuf: st["h"] = cmean(hbuf); st["hraw"] = hbuf[-1]
                if eL is not None and eR is not None:
                    eL = int(eL); eR = int(eR)
                    if lastE is not None:
                        dd = ((eL - lastE[0]) + (eR - lastE[1])) / 2.0 / TICKS_PER_M
                        st["x"] += dd * math.cos(math.radians(st["h"])); st["y"] += dd * math.sin(math.radians(st["h"]))
                    lastE = (eL, eR); st["encL"] = eL; st["encR"] = eR
                if s: st["scan"] = [None if v < 0 else v for v in s]
                if d0 is not None: st["d0"] = int(float(d0))
                if d5 is not None: st["d5"] = int(float(d5))
                if d11 is not None: st["d11"] = float(d11)
                if stt:
                    if stt.get("goal", 0) and not st["goal"]:
                        goal_pos = (st["x"], st["y"]); ev("*** GOAL=1 at pos (%.2f,%.2f) ***" % goal_pos)
                    st.update(tick=stt.get("tick", 0), goal=stt.get("goal", 0), here=stt.get("here", 0))
                st["t"] = time.time(); visited.add(cell(st["x"], st["y"])); snap = dict(st)
            with open(STATE, "w") as f: json.dump(snap, f)
            with open(LOG, "a") as f: f.write(json.dumps(snap) + "\n")
        except Exception as e:
            ev("sensor_loop error: %r" % e); time.sleep(0.2)

def rx_loop():
    while True:
        try:
            m = rb.read_line(10, 1.0)
            if m:
                with open(RXLOG, "a") as f: f.write("%.1f %s\n" % (time.time(), m))
                ev("RX: " + m)
        except Exception as e:
            ev("rx error %r" % e); time.sleep(0.5)

def scan_med(n=4):
    ss = []
    for _ in range(n):
        s = get("scan")
        if s: ss.append(s)
        time.sleep(0.12)
    out = []
    for i in range(16):
        v = [s[i] for s in ss if s[i] is not None]
        out.append(statistics.median(v) if v else 0.0)
    return out

def rot(target, tol=4, timeout=12):
    t0 = time.time(); st["busy"] = "rot"
    while time.time() - t0 < timeout:
        d = angdiff(target, get("h"))
        if abs(d) <= tol: break
        sp = 40 if abs(d) > 45 else (22 if abs(d) > 12 else 12)
        set_wheels(sp, -sp) if d > 0 else set_wheels(-sp, sp)
        time.sleep(0.06)
    set_wheels(0, 0); time.sleep(0.35)
    st["busy"] = ""; return get("h")

def drive2(target_h, max_dist=3.0, speed=80, timeout=45, stop_junction=True):
    """Heading-hold drive with gentle centering; stops at obstacle, junction, dist, stuck, goal."""
    st["busy"] = "drive"; x0, y0 = get("x"), get("y"); t0 = time.time(); reason = "dist"
    snap_pos = (x0, y0); snap_front = None; snap_t = time.time(); stuck = 0; stuck_ev = 0; tight_seen = False; bumps = 0; junc = 0
    while time.time() - t0 < timeout:
        if get("goal") or get("here"): reason = "GOAL"; break
        gone = math.hypot(get("x") - x0, get("y") - y0)
        if gone >= max_dist: break
        s = get("scan") or [None] * 16
        g = lambda i: s[i] if s[i] is not None else 9.0
        f0 = g(0); f1 = min(g(15), g(1))
        if f0 < 0.24 or f1 < 0.11: reason = "blocked f0=%.2f f1=%.2f" % (f0, f1); break
        if get("d5"):
            bumps += 1
            if bumps > 6: reason = "bump"; break
            # find nearest beam and nudge away: reverse while turning away from it
            near = min(range(16), key=lambda i: g(i))
            side = 1 if near <= 8 else -1     # +1: obstacle on +90 side -> decrease heading (turn away)
            set_wheels(-40 + 12 * side * -1, -40 - 12 * side * -1); time.sleep(0.45)
            set_wheels(0, 0); time.sleep(0.15); ev("bump nudge #%d near beam %d" % (bumps, near)); continue
        b4, b12 = g(4), g(12)
        if b4 < 0.45 and b12 < 0.45: tight_seen = True
        v4 = s[4] is not None and s[4] > 0.9; v12 = s[12] is not None and s[12] > 0.9
        junc = junc + 1 if (v4 or v12) else 0
        if stop_junction and tight_seen and gone > 0.25 and junc >= 2:
            reason = "junction b4=%.2f b12=%.2f" % (b4, b12); break
        # stuck detection
        if time.time() - snap_t > 0.6:
            fc = f0; moved = math.hypot(get("x") - snap_pos[0], get("y") - snap_pos[1])
            if snap_front is not None and moved > 0.03 and fc < 2.0 and snap_front < 2.0 and abs(fc - snap_front) < 0.012: stuck += 1
            else: stuck = 0
            if stuck >= 2:
                with lock: st["x"], st["y"] = snap_pos
                stuck_ev += 1; ev("STUCK #%d, backing off" % stuck_ev)
                set_wheels(-50, -50); time.sleep(0.6)
                if min(g(11), g(12), g(13)) < min(g(3), g(4), g(5)): set_wheels(25, -25)
                else: set_wheels(-25, 25)
                time.sleep(0.3); set_wheels(0, 0); time.sleep(0.2); stuck = 0
                if stuck_ev >= 3: reason = "stuck"; break
            snap_pos = (get("x"), get("y")); snap_front = fc; snap_t = time.time()
        sp = speed if f0 > 0.7 else 45
        corr = 1.0 * angdiff(target_h, get("h"))
        if b4 < 0.6 and b12 < 0.6: corr += max(-10, min(10, 60.0 * (b4 - b12)))
        mp = min(g(2), g(3), g(4)); mm = min(g(12), g(13), g(14))
        if mp < 0.2: corr -= 10 + 60 * (0.2 - mp)
        if mm < 0.2: corr += 10 + 60 * (0.2 - mm)
        if g(1) < 0.22: corr -= 12
        if g(15) < 0.22: corr += 12
        corr = max(-20, min(20, corr))
        set_wheels(sp + corr, sp - corr); time.sleep(0.07)
    set_wheels(0, 0); time.sleep(0.3); st["busy"] = ""
    gone = math.hypot(get("x") - x0, get("y") - y0)
    ev("drive2 done h=%.0f gone=%.2f reason=%s pos=(%.2f,%.2f) h=%.0f" % (target_h, gone, reason, get("x"), get("y"), get("h")))
    return reason

def openings(sm, thr=0.5):
    """list of (heading, range) for open directions from median scan"""
    h = get("h"); res = []
    for i in range(16):
        if sm[i] >= thr and sm[i] >= sm[(i - 1) % 16] and sm[i] >= sm[(i + 1) % 16]:
            # refine direction by neighbor weighting
            w = [(sm[(i + k) % 16], k) for k in (-1, 0, 1)]
            ang = sum(v * k for v, k in w) / max(1e-6, sum(v for v, k in w)) * 22.5
            res.append(((h + 22.5 * i + ang) % 360, sm[i]))
    return res

def hop(target_h, dist):
    rot(target_h); return drive2(target_h, dist)

def auto(n):
    came_from = None
    for k in range(n):
        if get("goal") or get("here"): ev("auto: goal/here reached, stop"); return
        # measure d11 for 8s
        vals = []
        for _ in range(40):
            vals.append(get("d11")); time.sleep(0.2)
            if get("here") or get("goal"): ev("auto: here/goal!"); return
        dv = sum(vals) / len(vals); samples.append((get("x"), get("y"), dv))
        with open("/tmp/d11samples.json", "w") as f: json.dump(samples, f)
        gx = gy = 0.0
        if len(samples) >= 3:
            # weighted least squares plane fit on recent samples
            rec = samples[-10:]; mx = sum(p[0] for p in rec) / len(rec); my = sum(p[1] for p in rec) / len(rec); mv = sum(p[2] for p in rec) / len(rec)
            sxx = sum((p[0]-mx)**2 for p in rec) + 1e-6; syy = sum((p[1]-my)**2 for p in rec) + 1e-6; sxy = sum((p[0]-mx)*(p[1]-my) for p in rec)
            sxv = sum((p[0]-mx)*(p[2]-mv) for p in rec); syv = sum((p[1]-my)*(p[2]-mv) for p in rec)
            det = sxx*syy - sxy*sxy
            if abs(det) > 1e-6: gx = (sxv*syy - syv*sxy) / det; gy = (syv*sxx - sxv*sxy) / det
        ev("auto: d11avg=%.3f grad=(%.2f,%.2f)" % (dv, gx, gy))
        sm = scan_med(); ops = openings(sm); x, y, h = get("x"), get("y"), get("h")
        junctions.append(dict(x=round(x, 2), y=round(y, 2), h=round(h), scan=[round(v, 2) for v in sm], ops=[(round(a), round(r, 2)) for a, r in ops]))
        with open("/tmp/junctions.json", "w") as f: json.dump(junctions, f)
        if not ops: ev("auto: no openings?!"); set_wheels(-40, -40); time.sleep(0.8); set_wheels(0, 0); continue
        def score(op):
            a, r = op; nx, ny = x + 0.8 * math.cos(math.radians(a)), y + 0.8 * math.sin(math.radians(a))
            sc = min(r, 2.5)
            if cell(nx, ny) not in visited: sc += 3.0
            nx2, ny2 = x + 1.3 * math.cos(math.radians(a)), y + 1.3 * math.sin(math.radians(a))
            if cell(nx2, ny2) not in visited: sc += 1.0
            if came_from is not None and abs(angdiff(a, came_from)) < 40: sc -= 2.5
            gn = math.hypot(gx, gy)
            if gn > 0.05:
                sc += 2.5 * (gx * math.cos(math.radians(a)) + gy * math.sin(math.radians(a))) / gn
            if target is not None:
                dn = math.hypot(target[0] - x, target[1] - y); dx = math.hypot(target[0] - nx, target[1] - ny)
                sc += 3.0 * (dn - dx)
            return sc
        ops.sort(key=score, reverse=True); a, r = ops[0]
        ev("auto hop %d: ops=%s -> choose h=%.0f r=%.2f" % (k, [(round(o[0]), round(o[1], 2)) for o in ops], a, r))
        reason = hop(a, min(r - 0.2, 0.7))
        came_from = (get("h") + 180) % 360
        if reason == "GOAL": ev("auto: GOAL reached"); return
        if reason == "bump": set_wheels(-40, -40); time.sleep(0.6); set_wheels(0, 0)

def cmd_loop():
    open(CMD, "a").close(); pos = os.path.getsize(CMD)
    while True:
        time.sleep(0.1)
        with open(CMD) as f:
            f.seek(pos); lines = f.readlines(); pos = f.tell()
        for line in lines:
            p = line.strip().split()
            if not p: continue
            ev("CMD: " + line.strip())
            try:
                if p[0] == "wheels": set_wheels(int(p[1]), int(p[2]))
                elif p[0] == "stop": set_wheels(0, 0)
                elif p[0] == "rot": rot(float(p[1]))
                elif p[0] == "hop": hop(float(p[1]), float(p[2]))
                elif p[0] == "auto": auto(int(p[1]))
                elif p[0] == "tx": rb.tx(line.strip()[3:]); ev("TX: " + line.strip()[3:])
                elif p[0] == "target":
                    global target
                    target = (float(p[1]), float(p[2])) if p[1] != "none" else None
                elif p[0] == "setpos":
                    with lock: st["x"], st["y"] = float(p[1]), float(p[2])
                elif p[0] == "quit": set_wheels(0, 0); os._exit(0)
            except Exception as e:
                ev("cmd error %r" % e); set_wheels(0, 0)

def load_visited():
    try:
        for l in open(LOG):
            try:
                r = json.loads(l); visited.add(cell(r["x"], r["y"]))
            except Exception: pass
    except Exception: pass

if __name__ == "__main__":
    load_visited()
    threading.Thread(target=sensor_loop, daemon=True).start()
    threading.Thread(target=rx_loop, daemon=True).start()
    ev("daemon v2 start"); cmd_loop()
