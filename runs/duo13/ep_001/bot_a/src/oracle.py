import sys, time, statistics, json
sys.path.insert(0,'/bot/src')
from robot import R
r=R(); r.motors(0,0)
def sample(n=7,to=0.06):
    vals=[]
    for _ in range(n*4):
        v=r.read(11,0.05)
        try: vals.append(float(v))
        except: pass
        if len(vals)>=n: break
        time.sleep(0.05)
    return statistics.median(vals) if vals else None
prev=None
t0=time.time()
lastreq=0
log=open('/memory/rx_all.log','a',buffering=1)
REQ="B1: reply 'B2VEC dx dy' = (GOAL minus B2pos) meters, compass frame x=cos(h)*d y=sin(h)*d. Or 'B2BRG brg dist' brg=deg from B2 nose. I hold still."
while True:
    m=r.read(10,0.4)
    now=time.time()
    if m:
        log.write(f"[{now:.0f}] {m}\n")
        if 'VEC' in m or 'BRG' in m or 'GOAL' in m.upper():
            open('/memory/vec_from_b1.txt','a').write(f"{now:.0f} {m}\n")
        if 'PING' in m.upper() or 'd11' in m or 'STOPPED' in m:
            cur=sample()
            if cur is None: continue
            if prev is None or abs(cur-prev)<0.015: word="SAME"
            elif cur>prev: word="WARMER"
            else: word="COLDER"
            r.write(8,f"B2 {word} d11={cur:.3f}")
            prev=cur
        else:
            r.write(8,f"B2 ACK d11={sample() or -1:.3f}")
    if now-lastreq>15:
        lastreq=now
        try:
            j=json.load(open('/memory/pose.json')); p=f"odo=({j['x']:.1f},{j['y']:.1f})DRIFTED"
        except: p=""
        r.write(8,REQ+p)
    if now-t0>60 and (now-t0)%120<1:
        r.write(8,"B2 STATUS: I am STILL, waiting for your B2VEC. Pinging 1/s.")
