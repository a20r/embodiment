import ctl, rb, time, math, json, os, heapq
import explore
from explore import grid, L, cell, mark_scan, sweep, frontier, obst_cost, save_map
ctl.CPU=1920.0
A_EST=(2.0,-1.5)
BIAS=2.0
laststatus=''
def plan(b):
    start=cell(b.x,b.y)
    dist={start:0}; prev={start:None}; pq=[(0,start)]
    best=None
    while pq:
        d,c=heapq.heappop(pq)
        if d>dist.get(c,1e9): continue
        if d>100000: break
        if c!=start and frontier(c) and obst_cost(c)<70 and abs(c[0]-start[0])+abs(c[1]-start[1])>=3:
            cx=(c[0]+0.5)*explore.CELL; cy=(c[1]+0.5)*explore.CELL
            score=d+BIAS*math.hypot(cx-A_EST[0],cy-A_EST[1])/explore.CELL
            if best is None or score<best[0]: best=(score,c)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            n=(c[0]+dx,c[1]+dy)
            if grid.get(n)!=1: continue
            nd=d+1+obst_cost(n)
            if nd<dist.get(n,1e9): dist[n]=nd; prev[n]=c; heapq.heappush(pq,(nd,n))
    if best is None: return None
    path=[]; c=best[1]
    while c is not None: path.append(c); c=prev[c]
    return path[::-1]
lastcomm=[0]
def comm(b,step,d11):
    global laststatus
    st=rb.rd('d3',0.3)
    core=st.split('tick=')[-1].split(' ',1)[-1].split(' tx')[0]
    if core!=laststatus: L('*** STATUS CHANGE',st); laststatus=core
    d0=rb.rd('d0',0.3)
    if d0.strip()!='0': L('*** d0 =',d0)
    if time.time()-lastcomm[0]>20:
        lastcomm[0]=time.time()
        rb.wr('d8',f'B: exploring, at my ({b.x:.1f},{b.y:.1f}) = your ({b.x+7.9:.1f},{b.y:.1f}). status {core}. d11={d11}')
        time.sleep(0.4); st=rb.rd('d3',0.3)
    for _ in range(8):
        rx=rb.rd('d10',0.3)
        if not rx.strip(): break
        open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {rx}\n')
        if 'GOAL' in rx.upper(): L('!!! RX GOAL:',rx[:200])
        elif 'homing' not in rx: L('RX:',rx[:160])
    return st
def main():
    b=ctl.Bot()
    for k in json.load(open('/bot/src/grid.json')): grid[(k[0],k[1])]=k[2]
    last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
    L(f'EXPLORE2 start {b.x:.2f},{b.y:.2f} h {b.h:.0f} grid {len(grid)}')
    step=0; nof=0
    while not os.path.exists('/bot/src/STOP'):
        step+=1
        d11=rb.rd('d11',0.3)
        st=comm(b,step,d11)
        path=plan(b)
        if not path:
            nof+=1; L('no frontier; sweep'); sweep(b,'nof')
            if nof>4: L('no frontiers left'); break
            continue
        nof=0
        idx=min(len(path)-1,4)
        tx=(path[idx][0]+0.5)*explore.CELL; ty=(path[idx][1]+0.5)*explore.CELL
        ang=math.degrees(math.atan2(ty-b.y,tx-b.x))%360; dist=math.hypot(tx-b.x,ty-b.y)
        L(f'step {step} pos {b.x:.2f},{b.y:.2f} h {b.h:.0f} d11 {d11} -> wp {tx:.2f},{ty:.2f} pathlen {len(path)} {st[-9:]}')
        b.turn_to(ang,tol=5); mark_scan(b,b.scan(),b.h)
        reason,trav=b.forward(dist,speed=45,minfront=0.14)
        if reason=='bump':
            c=cell(b.x+0.12*math.cos(math.radians(b.h)), b.y+0.12*math.sin(math.radians(b.h))); grid[c]=2
            b.set(-30,-30); time.sleep(0.6); b.stop(); b.update()
        elif reason.startswith('obstacle'): mark_scan(b,b.scan(),b.h)
        sweep(b,'e2_%d'%step); save_map()
    b.stop()
if __name__=='__main__':
    try: main()
    except Exception:
        import traceback; L('EXC',traceback.format_exc())
    finally: rb.wr('d1','0'); rb.wr('d7','0')
