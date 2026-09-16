import math, subprocess, time, os, json
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
def drive(a,b,dur,period=0.08):
    t0=time.time()
    while time.time()-t0<dur:
        wr('d1',a); wr('d7',b); time.sleep(period)
def d0rate(n=8):
    c=0
    for _ in range(n):
        if rd('d0')=='1': c+=1
        time.sleep(0.05)
    return c/n
log=open('/tmp/spiral.log','a',buffering=1)
t_end=time.time()+1150
step=0
last_ping=0
while time.time()<t_end:
    step+=1
    r=d0rate()
    fl=rd('d3'); d5=rd('d5')
    log.write(f'step{step} d0rate={r:.2f} d5={d5} {fl}\n')
    if 'here=1' in fl or 'goal=1' in fl or r>=0.9:
        log.write(f'*** HIT step{step} d0rate={r:.2f} {fl} — HOLDING\n')
        open('/memory/EVENT.log','a').write(json.dumps({'t':time.time(),'step':step,'d0rate':r,'fl':fl})+'\n')
        # hold here, blast radio
        t_hold=time.time()+240
        while time.time()<t_hold:
            if time.time()-last_ping>2: wr('d8','A GOAL SPOT COME'); last_ping=time.time()
            print('HOLD', rd('d3'), rd('d0'), flush=True)
            time.sleep(1.0)
            if 'here=1' in rd('d3'): 
                log.write('CONFIRMED HERE=1\n'); break
        continue
    # spirograph: rotate ~35 then creep ~8cm, alternating
    t0=time.time(); nh0=float(rd('d4')); tgt=None
    for _ in range(3):
        wr('d1',7); wr('d7',-7); time.sleep(0.9)
    wr('d1',0); wr('d7',0); time.sleep(0.2)
    drive(9,9,0.85)
    for _ in range(3): wr('d1',0); wr('d7',0)
    time.sleep(0.3)
    if time.time()-last_ping>3: wr('d8','A IN PEN SEARCH'); last_ping=time.time()
