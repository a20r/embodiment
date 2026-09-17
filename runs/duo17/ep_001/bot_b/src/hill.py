import ctl, rb, time, math, json, sys, os
import explore, goto
ctl.CPU=1920.0
grid=explore.grid
def d11(n=5):
    v=[]
    for _ in range(n):
        try: v.append(float(rb.rd('d11',0.3)))
        except: pass
        time.sleep(0.05)
    v.sort(); return v[len(v)//2] if v else 0
def ping(b,s):
    rb.wr('d8',f'Robot B: {s} my_signal={d11(3):.2f}. STAY PUT please, I am homing in on you. Any info on the goal?'); time.sleep(0.3)
    st=rb.rd('d3',0.3); rx=rb.rd('d10',0.3)
    if rx.strip(): open('/bot/src/msgs.txt','a').write(f'{time.time():.0f} RX: {rx}\n'); explore.L('RX:',rx)
    return st
def main():
    b=ctl.Bot()
    last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
    best=d11(); bestpos=(b.x,b.y)
    explore.L(f'HILL start {b.x:.2f},{b.y:.2f} d11 {best:.3f}')
    dirs=[0,45,90,135,180,225,270,315]
    tried={}
    for step in range(30):
        if os.path.exists('/bot/src/STOP'): break
        # candidate directions: free path 0.35 away
        cands=[]
        cc=explore.cell(b.x,b.y)
        for dx in range(-5,6):
            for dy in range(-5,6):
                c=(cc[0]+dx,cc[1]+dy)
                if grid.get(c)!=1 or explore.obst_cost(c)>=30: continue
                cx=(c[0]+0.5)*explore.CELL; cy=(c[1]+0.5)*explore.CELL
                d=math.hypot(cx-b.x,cy-b.y)
                if d<0.25 or d>0.45: continue
                ang=math.degrees(math.atan2(cy-b.y,cx-b.x))%360
                a=int(round(ang/45))*45%360
                cands.append((tried.get(a,0),a,cx,cy))
        if not cands:
            explore.L('no cands; sweep'); explore.sweep(b,'hill'); explore.save_map(); continue
        cands.sort(); _,a,tx,ty=cands[0]
        tried[a]=tried.get(a,0)+1
        x0,y0=b.x,b.y
        ang=math.degrees(math.atan2(ty-b.y,tx-b.x))%360
        b.turn_to(ang,tol=5); r,tr=b.forward(math.hypot(tx-b.x,ty-b.y),speed=45,minfront=0.14)
        explore.mark_scan(b,b.scan(),b.h)
        if r=='bump': b.set(-30,-30); time.sleep(0.5); b.stop()
        v=d11()
        st=ping(b,f'pos {b.x:.2f},{b.y:.2f}')
        explore.L(f'hill {step} dir {a} {r} {tr:.2f} -> {b.x:.2f},{b.y:.2f} d11 {v:.3f} (best {best:.3f}) {st[-8:]}')
        if v>best+0.01:
            best=v; bestpos=(b.x,b.y); tried={a:0}  # keep going this way
            tried={k:(0 if k==a else 1) for k in dirs}
        else:
            # go back toward best position if we lost signal
            if v<best-0.04:
                explore.L('  backtracking to best')
                goto.run(b,bestpos[0],bestpos[1],ping=False)
        explore.save_map(); b.record('hill')
    b.stop()
if __name__=='__main__':
    try: main()
    except Exception as e:
        import traceback; explore.L('EXC',traceback.format_exc())
    finally: rb.wr('d1','0'); rb.wr('d7','0')
