import time, json, math, threading, sys
from nav import Nav, CELL

logf = open('/bot/src/explore.log','a', buffering=1)
def log(*a):
    msg = ' '.join(str(x) for x in a)
    logf.write(f'[{time.strftime("%H:%M:%S")}] {msg}\n')

nav = Nav(log=log)
nav.sync_enc()
radio_replies = []

def radio():
    while True:
        nav.r.send(f'A PING {nav.x:.1f} {nav.y:.1f} {nav.hdg:.0f}')
        t0 = time.time()
        while time.time()-t0 < 6:
            v = nav.r.recv(0.5)
            if v:
                radio_replies.append(v)
                log('RADIO RX:', v)
                if 'GOAL' in v.upper():
                    log('RADIO MENTIONS GOAL!', v)
            time.sleep(0.2)

threading.Thread(target=radio, daemon=True).start()

log('=== explore start ===', nav.pose_str())
try:
    for round_i in range(60):
        log(f'--- round {round_i} pose={nav.pose_str()} batt={nav.r.battery()}')
        nav.full_scan()
        nav.update_pose()
        st = nav.r.status()
        if st and (st[1] or st[2]):
            log('GOAL FLAGS!', st)
            json.dump({'x':nav.x,'y':nav.y,'h':nav.hdg}, open('/memory/goal_found.json','w'))
            break
        start = nav.grid.key(nav.x, nav.y)
        fk, fn = nav.find_frontier(start)
        if fk is None:
            log('NO FRONTIER - done?')
            break
        path = nav.plan(start, fk)
        if path is None:
            log('NO PATH to frontier', fk)
            break
        log(f'frontier {fk}->{fn} path len {len(path)}')
        ok = nav.follow_path(path)
        log(f'follow ok={ok} pose={nav.pose_str()}')
        json.dump({str(k): v for k,v in nav.grid.occ.items()}, open('/bot/src/grid.json','w'))
except Exception as e:
    import traceback
    log('FATAL', traceback.format_exc())
    nav.r.stop()
log('=== explore end ===', nav.pose_str())
