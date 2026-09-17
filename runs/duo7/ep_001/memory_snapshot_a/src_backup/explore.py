import robot, time, math, json, sys, os

CPM = 1500.0   # encoder counts per meter (approx)
SPEED = 55
INF = 9.9

logf = open('/memory/trail.jsonl','a')

def lid():
    L = robot.lidar()
    return [x if x and x>0 else INF for x in L]

class Nav:
    def __init__(self):
        self.x=0.0; self.y=0.0
        self.e = robot.enc()
        self.h = robot.heading()
    def update(self):
        e = robot.enc(); h = robot.heading()
        d = ((e[0]-self.e[0])+(e[1]-self.e[1]))/2.0/CPM
        self.e = e; self.h = h
        r = math.radians(h)
        self.x += d*math.sin(r); self.y += d*math.cos(r)
        return h

nav = Nav()
last_log = 0
last_beacon = 0
t0 = time.time()

def mins(L, idxs):
    return min(L[i] for i in idxs)

goal_seen = None
try:
  while True:
    h = nav.update()
    L = lid()
    front = mins(L,[15,0,1])
    right = mins(L,[3,4,5])
    left  = mins(L,[11,12,13])
    now = time.time()

    st = robot.status() or ''
    if 'goal=1' in st:
        robot.motors(0,0)
        with open('/memory/GOAL_FOUND.txt','a') as f:
            f.write(json.dumps({'t':now,'x':nav.x,'y':nav.y,'st':st})+'\n')
        print('GOAL at', nav.x, nav.y, flush=True)
        goal_seen = (nav.x, nav.y)
        break

    # radio
    if now-last_beacon > 3:
        robot.tx('PING from=alpha x=%.2f y=%.2f'%(nav.x,nav.y))
        last_beacon = now
    msg = robot.readline('d4', timeout=0.05)
    if msg:
        with open('/memory/radio_rx.log','a') as f:
            f.write('%f %s\n'%(now,msg))
        print('RX:', msg, flush=True)

    if now-last_log > 0.7:
        d5 = robot.readline('d5',0.2); d2=robot.readline('d2',0.2); d9=robot.readline('d9',0.2)
        logf.write(json.dumps({'t':round(now-t0,1),'x':round(nav.x,2),'y':round(nav.y,2),
            'h':round(h,1),'L':[round(v,2) for v in L],'d5':d5,'d2':d2,'d9':d9})+'\n')
        logf.flush()
        last_log = now

    # right-hand wall following
    if front < 0.28:
        # turn left in place until front opens
        robot.motors(-25,25)
        time.sleep(0.12)
        continue
    # steer to keep right wall at ~0.25
    err = right - 0.25
    steer = max(-8, min(8, err*30))
    if right > 0.9: steer = 8   # right open: turn right into it
    if left < 0.14: steer = max(steer, 4)   # too close left
    if right < 0.14: steer = min(steer, -4)
    sp = SPEED if front > 0.5 else 30
    robot.motors(sp+steer, sp-steer)
    time.sleep(0.05)
finally:
  robot.motors(0,0)
