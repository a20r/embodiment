import sys, time, math; sys.path.insert(0, '/bot/src')
import explore as E
from explore import *
E.POSE = pose = Pose(0.0, 0.0)
t0 = time.time(); n = 0
while time.time() - t0 < 900:
    s = read_line(3, 0.3) or ''
    if 'here=1' in s: stop(); log('NW: reached goal here=1', pose.get()); break
    x, y, th = pose.get()
    # alternate bias: mostly NW, sometimes pure N or pure W to get around walls
    ang = [135, 90, 180, 135][n % 4]; n += 1
    reactive_step(pose, x + 3*math.cos(math.radians(ang)), y + 3*math.sin(math.radians(ang)), steplen=0.35)
    if n % 6 == 0:
        v = read_line(11, 0.3); log('NW step %d pose=(%.2f,%.2f) d11=%s %s' % (n, x, y, v, s))
        write_line(8, 'B: heading back to goal (NW corner room). Follow rising d11 / route. here=1 marks goal.')
while True:
    s = read_line(3, 0.3) or ''; v = read_line(11, 0.3)
    write_line(8, 'B: PARKED ON GOAL (here=1), NW corner room. Come here: keep d11 rising. My d11=%s. %s' % (v, s))
    log('PARK %s d11=%s' % (s, v))
    if 'goal=1' in s: log('GOAL=1 !!!'); break
    time.sleep(5)
