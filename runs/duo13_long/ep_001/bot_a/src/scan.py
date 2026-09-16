import time, math, json
from robot import Robot
r = Robot()
r.stop(); time.sleep(0.5)
scan = []
h0 = r.heading()
for i in range(24):
    h = r.heading()
    lid = r.lidar()
    st = r.status()
    scan.append({'h': h, 'lid': lid, 'goal': st[1], 'here': st[2], 'd0': r.rd('d0'), 'd5': r.rd('d5')})
    r.rot(15, speed=25)
    time.sleep(0.3)
h = r.heading()
lid = r.lidar()
scan.append({'h': h, 'lid': lid, 'goal': r.status()[1], 'here': r.status()[2], 'd0': r.rd('d0'), 'd5': r.rd('d5')})
json.dump(scan, open('/bot/src/scan1.json','w'), indent=1)
for s in scan:
    if s['lid']:
        mx = max(s['lid'])
        print(f"h={s['h']:6.1f} goal={s['goal']} here={s['here']} d0={s['d0']} d5={s['d5']} max={mx:.2f}@{s['lid'].index(mx)} lid={['%.2f'%x for x in s['lid']]}")
