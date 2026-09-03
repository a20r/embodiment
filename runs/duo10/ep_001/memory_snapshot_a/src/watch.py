import time, select
D='/dev/robot/'
out=open('/bot/src/watch.log','a')
def rd(p,t=0.6):
    try:
        f=open(D+p); r,_,_=select.select([f],[],[],t)
        if not r: f.close(); return None
        s=f.readline().strip(); f.close(); return s
    except: return None
last=('','','')
while True:
    d2=rd('d2'); d9=rd('d9'); d6=rd('d6')
    g=''
    if d6:
        parts=dict(kv.split('=') for kv in d6.split())
        g=f"goal={parts.get('goal')} here={parts.get('here')}"
    cur=(d2,d9,g)
    if (d2 not in ('0',None)) or (d9 not in ('0',None)) or ('here=1' in g) or ('goal=1' in g) or ('goal=2' in g):
        print(time.strftime('%H:%M:%S'), 'ALERT', d2, d9, d6, file=out); out.flush()
    time.sleep(1)
