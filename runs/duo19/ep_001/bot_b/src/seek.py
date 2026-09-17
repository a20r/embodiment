import sys, time, math; sys.path.insert(0,'/bot/src')
from drive import *
from rob import read_line, write_line
def here():
    s = read_line(3, 0.3) or ''
    return 'here=1' in s, s
def drive_until_here(bearing, maxd=1.0, speed=40):
    turn_to(bearing, tol=4)
    e0 = enc(); t0 = time.time()
    while time.time() - t0 < 12:
        h, s = here()
        if h: stop(); return 'HERE', s
        r = ranges()
        if r:
            f = [v for v in (r[0], r[1], r[15]) if v > 0]
            if f and min(f) < 0.14: stop(); return 'obstacle', s
        e = enc()
        if e and e0 and ((e[0]-e0[0])+(e[1]-e0[1]))/2/1820.0 >= maxd: stop(); return 'maxd', s
        motors(speed, speed); time.sleep(0.03)
    stop(); return 'timeout', s
if __name__ == '__main__':
    b = float(sys.argv[1]); d = float(sys.argv[2])
    print(drive_until_here(b, d))
    h = heading(); r = ranges()
    print('h', h, 'ranges', ' '.join('%d:%.2f' % (i, v) for i, v in enumerate(r)))
