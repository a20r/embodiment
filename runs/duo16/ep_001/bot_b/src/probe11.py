import sys, time, json, math
sys.path.insert(0,'/bot/src')
from rob import *
def lastline(name, timeout=0.5, tries=5):
    for i in range(tries):
        d = read_port(name, timeout).decode(errors='replace')
        lines=[l for l in d.strip().split('\n') if l.strip()]
        if lines: 
            try: return float(lines[-1])
            except: pass
    return None
def drive(l,r,secs):
    write_port('d1',str(l)); write_port('d7',str(r)); time.sleep(secs)
    write_port('d1','0'); write_port('d7','0')
def avgd11(n=6):
    vs=[]
    for i in range(n):
        v=lastline('d11',0.3,3)
        if v is not None: vs.append(v)
        time.sleep(0.06)
    return sum(vs)/len(vs) if vs else None
def setheading(target):
    for attempt in range(6):
        h=lastline('d4')
        if h is None: time.sleep(0.2); continue
        diff=(target-h+180)%360-180
        if abs(diff)<4: return h
        t=min(abs(diff),60)/8.0
        if diff>0: drive(8,-8,t)
        else: drive(-8,8,t)
        time.sleep(0.3)
    return lastline('d4')
print('start d11 avg:', round(avgd11() or -1,4), 'heading', lastline('d4'))
res={}
for target in [0,90,180,270]:
    h=setheading(target)
    time.sleep(0.5)
    d0=avgd11()
    o9=lastline('d9'); o6=lastline('d6')
    drive(25,25,1.5)
    time.sleep(0.3)
    o9b=lastline('d9'); o6b=lastline('d6')
    d1=avgd11()
    ticks=((o9b-o9)+(o6b-o6))/2 if None not in (o9,o6,o9b,o6b) else -1
    res[target]=(h, d0, d1, (d1-d0) if None not in (d0,d1) else None, ticks)
    print('heading %s: d11 %s -> %s (d=%s) ticks=%s' % (target, round(d0,4) if d0 else None, round(d1,4) if d1 else None, round(d1-d0,4) if None not in (d0,d1) else None, ticks), flush=True)
json.dump(res, open('/memory/grad.json','w'))
print('d0=',lastline('d0'),'d5=',lastline('d5'))
