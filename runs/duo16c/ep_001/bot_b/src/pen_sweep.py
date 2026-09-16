import math, subprocess, time, os, json, threading
def wr(p,v):
    for att in range(3):
        try:
            fd=os.open(f'/dev/robot/{p}',os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd,f'{int(v)}\n'.encode()); os.close(fd); return
        except Exception: time.sleep(0.03)
def rd(p,tries=4):
    for _ in range(tries):
        r=subprocess.run(['timeout','0.3','cat',f'/dev/robot/{p}'],capture_output=True,text=True)
        if r.stdout.strip(): return r.stdout.strip()
        time.sleep(0.04)
    return '0'
# continuous d0 logger
hits=[]
stop_flag=threading.Event()
def d0_logger():
    while not stop_flag.is_set():
        v=rd('d0')
        hits.append((time.time(), v))
        time.sleep(0.06)
th=threading.Thread(target=d0_logger, daemon=True); th.start()
log=open('/tmp/pen_sweep.log','a',buffering=1)
def drive(a,b,dur,period=0.08):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(period)
        # check recent hits
        recent=[v for t,v in hits[-25:]]
        if recent.count('1')>=8:
            return 'HIT'
    return None
t_end=time.time()+1000
last_ping=0
ring=0
try:
    while time.time()<t_end:
        # rotate ~75 deg slowly
        res=drive(7,-7,3.2)
        if res=='HIT':
            log.write('*** HIT during rotate ***\n'); break
        for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.15)
        # creep out 12cm
        res=drive(11,11,1.6)
        if res=='HIT':
            log.write('*** HIT during creep ***\n'); break
        for _ in range(3): wr('d1',0); wr('d7',0); time.sleep(0.15)
        ring+=1
        if time.time()-last_ping>4: wr('d8','A SWEEPING PEN'); last_ping=time.time()
        log.write(f'ring{ring} hits_tail={hits[-20:].count((hits[-1][0],"1")) if hits else 0}\n')
        # avoid wall: if pressed (fwd blocked) back off handled implicitly
finally:
    stop_flag.set()
    for _ in range(4): wr('d1',0); wr('d7',0)
    json.dump(hits[-4000:], open('/memory/d0_hits.json','w'))
    # analysis
    t_now=time.time()
    ones=[t for t,v in hits if v=='1' and t>t_now-600]
    log.write(f'total ones in window: {len(ones)}\n')
    if ones:
        log.write(f'last one at {ones[-1]:.1f} ({t_now-ones[-1]:.1f}s ago)\n')
