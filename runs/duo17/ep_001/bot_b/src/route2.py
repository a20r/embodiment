import ctl,rb,time,json,math
import leg
from explore import L
ctl.CPU=1920.0
b=ctl.Bot(); last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
def rng(s,A):
    k=int(round(((A-b.h)%360)/22.5))%16; return s[k]
def go(H,maxd,cond=None,step=0.15,mind=0.0):
    b.turn_to(H,tol=4); trav=0
    while trav<maxd:
        r,tr=b.forward(min(step,maxd-trav),speed=36,minfront=0.16); trav+=tr
        s=b.scan(); st=rb.rd('d3',0.3); d=leg.d11(3)
        e,w,n,so=rng(s,0),rng(s,180),rng(s,90),rng(s,270)
        L(f'r2 H{H} ({b.x:.2f},{b.y:.2f}) E{e:.2f} W{w:.2f} N{n:.2f} S{so:.2f} d11 {d:.2f} {st.split(" ",1)[1].split(" tx")[0]}')
        if 'here=1' in st: L('!!!!! HERE=1 ARRIVED'); return 'HERE'
        if r!='done': return r
        if cond and trav>=mind and cond(e,w,n,so): return 'cond'
    return 'done'
try:
    b.set(-28,-28); time.sleep(0.5); b.stop()
    L('r2 step A: east 0.25 then south into pocket corridor')
    go(0,0.25)
    r=go(270,1.2,lambda e,w,n,so: e>0.8 and w>0.8, mind=0.2); L('r2 south->',r)
    # now in E-W pocket corridor: go east until long view south
    r=go(0,3.5,lambda e,w,n,so: so>1.6, mind=0.5); L('r2 east->',r)
    if r=='cond':
        rb.wr('d8','B: found long southward corridor, descending to you now.')
        r=go(270,3.0); L('r2 south2->',r)
        if r!='HERE':
            # try small adjustments: east/west nook
            s=b.scan(); L('r2 at bottom scan', ' '.join(f'{(b.h+22.5*k)%360:.0f}:{d:.2f}' for k,d in enumerate(s)))
    L('r2 end', rb.rd('d3'), 'd11', leg.d11(3))
finally: rb.wr('d1','0'); rb.wr('d7','0')
