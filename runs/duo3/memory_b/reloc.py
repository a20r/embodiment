import re,json,sys
def parse(path,last_run=True):
    runs=[]; cur=None
    for line in open(path):
        if 'START' in line or 'AGENT start' in line:
            cur={'cells':{}}; runs.append(cur); continue
        if cur is None: cur={'cells':{}}; runs.append(cur)
        m=re.search(r'CELL \((-?\d+),\s*(-?\d+)\) f=(\d+) open=\[([^\]]*)\]',line)
        if m:
            x,y=int(m.group(1)),int(m.group(2))
            opens=frozenset(int(v) for v in m.group(4).split(',') if v.strip())
            cur['cells'].setdefault((x,y),[]).append(opens)
    return runs
old=json.load(open('/memory/ep1_bigrun.json'))
old={tuple(map(int,k.split(','))):set(v) for k,v in old.items()}
runs=parse('/memory/grid.log')
cur=max(runs,key=lambda r:len(r['cells']))['cells']
cur={k:set().union(*v) for k,v in cur.items()}
best=[]
for dx in range(-25,26):
    for dy in range(-25,26):
        sc=0; n=0
        for (x,y),o in cur.items():
            k=(x+dx,y+dy)
            if k in old:
                n+=1
                inter=len(o&old[k]); diff=len(o^old[k])
                sc+=inter-diff
        if n>=8: best.append((sc,n,dx,dy))
best.sort(reverse=True)
for b in best[:5]: print(b)
