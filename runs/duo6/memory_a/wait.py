import time, json
DEV='/dev/robot/'
def rd(p):
    try:
        with open(DEV+p) as f: return f.readline().strip()
    except: return ''
def wr(p,v):
    with open(DEV+p,'w') as f: f.write(v+'\n')
LOG=open('/memory/telemetry.jsonl','a')
def log(r):
    r['t']=round(time.time(),1); LOG.write(json.dumps(r)+'\n'); LOG.flush()
last=0
while True:
    s=rd('d4')
    if s:
        log({'rx':s})
    now=time.time()
    if now-last>3:
        d5=rd('d5'); d6=rd('d6')
        wr('d0',f'B: HOLDING STILL, you home to me. my d5={d5}')
        log({'st':'hold','d5':d5,'d6':d6,'l':rd('d3')})
        last=now
    time.sleep(0.25)
