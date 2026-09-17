import sys, time, math, json; sys.path.insert(0, '/bot/src')
import explore as E
from explore import *
from seek import drive_until_here
def d11():
    vs = []
    for _ in range(3):
        try: vs.append(float(read_line(11, 0.3)))
        except: pass
    return sum(vs)/len(vs) if vs else 0.0
def goto(pose, tx, ty, tmax=150, pause_for_A=False):
    tc = cell(tx, ty); t0 = time.time()
    while time.time() - t0 < tmax:
        x, y, th = pose.get()
        if math.hypot(tx-x, ty-y) < 0.15: return True
        scan_into_grid((x, y, th)); hard, soft = cost_maps(); start = cell(x, y); score[start] = min(score.get(start, 0), -1)
        path = plan(start, hard, soft, lambda c: math.hypot(c[0]-tc[0], c[1]-tc[1]) <= 3)
        if not path: reactive_step(pose, tx, ty); continue
        i = 0
        while i < len(path) - 1:
            j = min(len(path)-1, i+(3 if pause_for_A else 6))
            ok = go_to_cell(pose, path[j], speed=40); scan_into_grid(pose.get())
            if not ok: reactive_step(pose, tx, ty); break
            i = j
            if pause_for_A:
                w0 = time.time()
                while d11() < 0.85 and time.time() - w0 < 60: time.sleep(1)
                if d11() < 0.7:
                    log('LEAD lost A (d11 %.2f), re-climbing' % d11()); climb(pose, tmax=180); return goto(pose, tx, ty, tmax, True)
                log('LEAD seg d11=%.2f waited %.0fs' % (d11(), time.time()-w0))
    return False
def climb(pose, tmax=420):
    t0 = time.time(); bad = {}; last_dir = None
    while time.time() - t0 < tmax:
        v0 = d11()
        if v0 > 0.9: log('CLIMB close enough d11=%.2f' % v0); return True
        x, y, th = pose.get(); r = ranges() or [0]*16
        cands = []
        for i in range(16):
            ri = r[i] if r[i] > 0 else 0.25
            if ri < 0.45: continue
            b = (th + 22.5*i) % 360; key = int(b // 45)
            sc = -min(ri, 1.5) + bad.get(key, 0) * 0.8 + (0 if last_dir is None else min(abs(angdiff(b, last_dir)), 90) / 90.0)
            cands.append((sc, b, ri))
        if not cands:
            i = max(range(16), key=lambda k: r[k]); cands = [(0, th + 22.5*i, r[i])]
        cands.sort(); sc, b, ri = cands[0]
        tx = x + 0.5*math.cos(math.radians(b)); ty = y + 0.5*math.sin(math.radians(b))
        reactive_step(pose, tx, ty, steplen=0.3)
        v1 = d11(); key = int((b % 360) // 45)
        log('CLIMB dir %.0f d11 %.2f->%.2f pose=(%.2f,%.2f)' % (b, v0, v1, *pose.get()[:2]))
        if v1 < v0 + 0.012: bad[key] = bad.get(key, 0) + (1 if v1 < v0 - 0.015 else 0.6); last_dir = None
        else: last_dir = b; bad[key] = max(0, bad.get(key, 0) - 0.5)
        if time.time() - t0 > 60 and v1 > 0.5:
            write_line(8, 'B: I am coming to you to lead you. My d11=%.2f. Stay put & keep d11 rising toward me when I stop.' % v1)
    return False
def main():
    E.POSE = pose = Pose(float(sys.argv[1]), float(sys.argv[2]))
    for k, v in json.load(open('/bot/src/grid.json')).items():
        a, b = k.split(','); score[(int(a), int(b))] = v
    gx, gy = float(sys.argv[3]), float(sys.argv[4])
    time.sleep(0.3); log('FETCH start', pose.get())
    if len(sys.argv) < 6: goto(pose, -3.8, 3.6); log('FETCH at meeting point d11=%.2f' % d11())
    climb(pose)
    log('FETCH leading back from', pose.get(), 'd11=%.2f' % d11())
    write_line(8, 'B: FOLLOW ME now: keep your d11 > 0.8 (stay within 0.7 of me). I move slowly to the goal and pause for you.')
    goto(pose, gx, gy, tmax=1500, pause_for_A=True)
    for b, d in ((60, 0.5), (240, 0.6), (150, 0.5), (330, 0.5)):
        s = read_line(3) or ''
        if 'here=1' in s: break
        res = drive_until_here(b, d, speed=35); log('seek', b, res)
        if res[0] == 'HERE': break
    log('FETCH parked', pose.get(), read_line(3), 'd11=%.2f' % d11()); save_map()
    for _ in range(600):
        s = read_line(3) or ''; v = d11()
        write_line(8, 'B: parked on goal (here=1). Come to me: raise d11. d11 now %.2f. %s' % (v, s))
        if 'goal=1' in s: log('GOAL=1 !!!', s); break
        time.sleep(5)
if __name__ == '__main__':
    try: main()
    except Exception: import traceback; log('EXC', traceback.format_exc())
    finally: stop(); log('FETCH EXIT')
