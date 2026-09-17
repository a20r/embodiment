import time, math, select, random, collections
exec(open('/bot/src/explore2.py').read().split("log('=== explore2 start ===')")[0].replace('log4','log5'))
# we now have: rd, wr, lidar, rdf, drive, Bot b, visits, cell, turn_to, leg, choose, check_events, log, MAP
log('=== leader sweep start ===')
def d5v():
    v=rdf('d5'); return v if v is not None else 0.0
while True:
    # wait for botB if needed
    v=d5v()
    if v<0.65:
        drive(0,0)
        wr('d0', f'botA WAITING for you, climb d5. my d5={v:.2f}')
        log(f'waiting for botB d5={v:.2f}')
        t0=time.time()
        while time.time()-t0<40:
            v=d5v()
            st=rd('d6')
            if check_events(st): break
            if v>0.80: break
            time.sleep(2)
        if False:
            # backtrack toward botB: climb d5 ourselves briefly
            log('backtracking toward botB')
            for _ in range(6):
                # greedy: try beam most aligned with d5 increase via short probes
                t=choose()  # fallback: novelty (bad) -> instead pick random open beam
                l=lidar()
                if not l: continue
                k=max(range(16), key=lambda i: min(l[i], l[(i+1)%16]+0.3, l[(i-1)%16]+0.3))
                v0=d5v()
                turn_to((b.h+22.5*k)%360)
                leg((b.h)%360, maxdist=0.5)
                if d5v()>v0+0.03: continue
        continue
    t=choose()
    if t is None:
        drive(-30,-30); time.sleep(1); drive(0,0); continue
    turn_to(t)
    r=leg(t, maxdist=1.0, sp=50)
    if r=='GOAL':
        while True:
            wr('d0','GOAL FOUND here=1! COME HERE NOW: climb d5 to me and stand on my spot.')
            log('at goal, waiting: '+str(rd('d6')))
            time.sleep(4)
    drive(0,0)
    wr('d0', f'botA leg done, pausing. d5={d5v():.2f} follow me (climb d5).')
    time.sleep(2)
