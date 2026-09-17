import ctl,rb,time,json
import leg
from explore import L
ctl.CPU=1920.0
b=ctl.Bot(); last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
def rng(s,A):
    k=int(round(((A-b.h)%360)/22.5))%16; return s[k]
def go(H,maxd,cond=None,step=0.15,mind=0.0):
    b.turn_to(H,tol=4); trav=0
    while trav<maxd:
        r,tr=b.forward(min(step,maxd-trav),speed=34,minfront=0.16); trav+=tr
        s=b.scan(); st=rb.rd('d3',0.3); d=leg.d11(3)
        e,w,n,so=rng(s,0),rng(s,180),rng(s,90),rng(s,270)
        L(f'r3 H{H} ({b.x:.2f},{b.y:.2f}) E{e:.2f} W{w:.2f} N{n:.2f} S{so:.2f} d11 {d:.2f} {st.split(" ",1)[1].split(" tx")[0]}')
        if 'here=1' in st: L('!!!!! HERE=1 ARRIVED'); return 'HERE'
        if r!='done': return r
        if cond and trav>=mind and cond(e,w,n,so): return 'cond'
    return 'done'
try:
    go(180,0.12)  # center on the S opening
    r=go(270,1.2,lambda e,w,n,so: e>0.5,mind=0.3); L('r3 S1->',r)
    if r!='HERE':
        r=go(0,1.0,lambda e,w,n,so: so>0.6,mind=0.25); L('r3 E1->',r)
    if r!='HERE':
        r=go(270,2.5); L('r3 S2->',r)
    s=b.scan(); L('r3 final scan',' '.join(f'{(b.h+22.5*k)%360:.0f}:{d:.2f}' for k,d in enumerate(s)))
    L('r3 end',rb.rd('d3'),'d11',leg.d11(3)); b.record('r3end')
finally: rb.wr('d1','0'); rb.wr('d7','0')
