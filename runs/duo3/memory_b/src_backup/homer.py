import time, json, math, re, collections
from grid3 import Nav, Explorer, DIRS
from ctl import angdiff

ex=Explorer()          # fresh log handles
b=ex.b
d=json.load(open('/memory/grid.json'))
vis={tuple(c) for c in map(tuple,d['visited'])}
walls={tuple(map(int,k.split(','))):v for k,v in d['walls'].items()}
ex.cx,ex.cy=d['pos']
h=b.heading(); ex.facing=round(h/90)%4*90
lg=ex.lg
lg(f'HOMER start pos={d["pos"]} facing={ex.facing}')

svals=collections.deque(maxlen=20)
def poll_radio(tag=''):
    out=[]
    for m in b.radio_recv():
        lg(f'RX{tag}: {m}')
        mm=re.search(r's=([0-9.]+)',m)
        if mm: svals.append((time.time(),float(mm.group(1))))
        out.append(m)
    return out

def edge_open(a,bb):
    da=(bb[0]-a[0],bb[1]-a[1])
    dirn=[k for k,v in DIRS.items() if v==da][0]
    wa=walls.get(a,{}); wb=walls.get(bb,{})
    o1=wa.get(str(dirn)); o2=wb.get(str((dirn+180)%360))
    return (o1 is True) or (o2 is True)

def bfs(src,dst):
    q=collections.deque([src]); prev={src:None}
    while q:
        c=q.popleft()
        if c==dst: break
        x,y=c
        for dirn,(dx,dy) in DIRS.items():
            n=(x+dx,y+dy)
            if n in vis and n not in prev and edge_open(c,n):
                prev[n]=c; q.append(n)
    if dst not in prev: return None
    path=[]; c=dst
    while c is not None: path.append(c); c=prev[c]
    return path[::-1]

target=(3,-4)
for attempt in range(12):
    path=bfs((ex.cx,ex.cy),target)
    if path is None:
        lg('no path!'); break
    lg(f'path len {len(path)}')
    ok=True
    for nxt in path[1:]:
        b.radio_send('A: B HOLD STILL and keep beaconing s=, I am coming to you')
        poll_radio()
        dxy=(nxt[0]-ex.cx,nxt[1]-ex.cy)
        dirn=[k for k,v in DIRS.items() if v==dxy][0]
        if ex.go(dirn):
            ex.cx,ex.cy=nxt; ex.facing=dirn
        else:
            lg(f'go fail toward {dirn} at {(ex.cx,ex.cy)}')
            walls.setdefault((ex.cx,ex.cy),{})[str(dirn)]=False
            walls.setdefault(nxt,{})[str((dirn+180)%360)]=False
            ok=False; break
        if len(svals)>=3 and svals[-1][1]>0.995:
            lg('signal very high, stop path-following'); ok=True; break
    if ok: break
lg(f'HOMER done at {(ex.cx,ex.cy)}')
b.stop()
# sit and listen a bit, report
t0=time.time()
while time.time()-t0<20:
    poll_radio(' post')
    b.radio_send(f'A: at old-signal-peak zone, cell {(ex.cx,ex.cy)}. Send s= values.')
    time.sleep(2)
lg(f'svals recent: {[round(v,3) for _,v in list(svals)[-10:]]}')
