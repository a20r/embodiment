import time, json
DEV='/dev/robot/'
def rd(p):
    try:
        with open(DEV+p) as f: return f.readline().strip()
    except: return ''
def wr(p,v):
    with open(DEV+p,'w') as f: f.write(v+'\n')
LOG=open('/memory/radio.jsonl','a')
last=0
while True:
    s=rd('d4')
    now=time.time()
    if s:
        LOG.write(json.dumps({'t':round(now,1),'rx':s})+'\n'); LOG.flush()
        if now-last>2:
            wr('d0','B: I hear you, bot A. This is bot B. Do you know where the goal is? Reply GOAL x y or say NO. I will follow you.')
            last=now
    else:
        time.sleep(0.3)
