import ctl, rb, time, math, sys, json
ctl.CPU=1920.0
# usage: goto.py x y  (in explorer frame) -- uses saved grid for planning
import explore
grid=explore.grid
for k in json.load(open('/bot/src/grid.json')): grid[(k[0],k[1])]=k[2]
class B2(ctl.Bot):
    pass
def path_to(b,tx,ty):
    import heapq
    start=explore.cell(b.x,b.y); goal=explore.cell(tx,ty)
    dist={start:0}; prev={start:None}; pq=[(0,start)]
    while pq:
        d,c=heapq.heappop(pq)
        if c==goal: break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            n=(c[0]+dx,c[1]+dy)
            if grid.get(n)!=1: continue
            nd=d+1+explore.obst_cost(n)
            if nd<dist.get(n,1e9): dist[n]=nd; prev[n]=c; heapq.heappush(pq,(nd,n))
    if goal not in prev: return None
    p=[]; c=goal
    while c is not None: p.append(c); c=prev[c]
    return p[::-1]
def run(b,tx,ty,ping=True):
    for step in range(40):
        p=path_to(b,tx,ty)
        if not p: explore.L('goto: no path'); return False
        if len(p)<=1: return True
        idx=min(len(p)-1,4)
        wx=(p[idx][0]+0.5)*explore.CELL; wy=(p[idx][1]+0.5)*explore.CELL
        ang=math.degrees(math.atan2(wy-b.y,wx-b.x))%360; d=math.hypot(wx-b.x,wy-b.y)
        b.turn_to(ang,tol=5)
        r,tr=b.forward(d,speed=45,minfront=0.14)
        s=b.scan(); explore.mark_scan(b,s,b.h)
        d11=rb.rd('d11',0.3)
        st=''
        if ping:
            rb.wr('d8',f'HELLO robot A from robot B. I hear you. STOP and stay put, I will come to you. My d11 signal={d11}. Reply with anything you know about the goal.')
            time.sleep(0.3); st=rb.rd('d3',0.3); rx=rb.rd('d10',0.3)
            if rx.strip(): open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {rx}\n'); explore.L('RX:',rx)
        explore.L(f'goto step {step} pos {b.x:.2f},{b.y:.2f} h {b.h:.0f} fwd {r} {tr:.2f} d11 {d11} {st[-10:]}')
        if r=='bump':
            b.set(-30,-30); time.sleep(0.5); b.stop()
        if math.hypot(tx-b.x,ty-b.y)<0.15: return True
    return False
if __name__=='__main__':
    b=ctl.Bot()
    # restore pose from last explorer pose
    last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1])
    b.x,b.y=last['x'],last['y']
    explore.L('goto start from',b.x,b.y,b.h,'->',sys.argv[1],sys.argv[2])
    try:
        ok=run(b,float(sys.argv[1]),float(sys.argv[2]))
        explore.L('goto done',ok)
        b.record('goto_end')
        explore.save_map()
    finally:
        rb.wr('d1','0'); rb.wr('d7','0')
