import ctl,rb,time,json,math,sys
ctl.CPU=1920.0
def d11(n=5):
    v=[]
    for _ in range(n):
        try: v.append(float(rb.rd('d11',0.3)))
        except: pass
    v.sort(); return v[len(v)//2] if v else -1
def status(): return rb.rd('d3',0.3)
def leg(b,H,maxd,stop_open=None,step=0.15):
    """drive along heading H up to maxd; stop if blocked; if stop_open=('E'|'W'|'N'|'S', mindist) stop when that side opens (>0.45) after mindist"""
    b.turn_to(H,tol=4); trav=0; out=[]
    while trav<maxd:
        r,tr=b.forward(min(step,maxd-trav),speed=38,minfront=0.16); trav+=tr
        s=b.scan(); st=status()
        # absolute-direction ranges
        def rng(A):
            k=int(round(((A-b.h)%360)/22.5))%16; return s[k]
        e,w,n,so=rng(0),rng(180),rng(90),rng(270)
        out.append(f'({b.x:.2f},{b.y:.2f}) E{e:.2f} W{w:.2f} N{n:.2f} S{so:.2f} d11 {d11(3):.2f} {st.split(" ",1)[1].split(" tx")[0]}')
        if 'here=1' in st: out.append('!!! HERE=1'); break
        if r!='done': out.append('stop:'+r); break
        if stop_open and trav>=stop_open[1]:
            side={'E':e,'W':w,'N':n,'S':so}[stop_open[0]]
            if side>0.45: out.append('opening '+stop_open[0]); break
    b.record('leg'); return out
if __name__=='__main__':
    b=ctl.Bot(); last=json.loads(open('/bot/src/pose.jsonl').readlines()[-1]); b.x,b.y=last['x'],last['y']
    H=float(sys.argv[1]); maxd=float(sys.argv[2])
    so=(sys.argv[3],float(sys.argv[4])) if len(sys.argv)>4 else None
    try:
        for line in leg(b,H,maxd,so): print(line)
    finally: rb.wr('d1','0'); rb.wr('d7','0')
