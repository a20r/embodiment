src=open('/bot/src/explore5.py').read()
a="""t0=time.time(); lastlog=0; th_t=None
consec_turns=0; last_turn_tick=0"""
b="""t0=time.time(); lastlog=0; th_t=None
consec_turns=0; last_turn_tick=0
turnpref=-1
sigs=[]"""
src=src.replace(a,b)
a2="""def pick(b):
    best=None;bs=-9
    for i in range(16):
        ang=abs(i-8)*22.5
        sc=b[i]-ang*0.008
        if sc>bs: bs=sc;best=i
    return best"""
b2="""def pick(b):
    best=None;bs=-9
    for i in range(16):
        ang=abs(i-8)*22.5
        sc=b[i]-ang*0.008+(0.25 if (i-8)*turnpref>0 else 0)
        if sc>bs: bs=sc;best=i
    return best
def note_sig(b):
    s=tuple(int(round(x*4)) for x in b)
    sigs.append(s)
    if len(sigs)>14: sigs.pop(0)
    return sigs.count(s)"""
src=src.replace(a2,b2)
a3="""        best=pick(b); err=best-8
        if err==0: err=-2
        log("TURN best=%d err=%d ticks=%d"%(best,err,tk))"""
b3="""        cnt=note_sig(b)
        if cnt>=3:
            turnpref*=-1; sigs.clear()
            log("LOOP! switch side -> %d"%turnpref)
        best=pick(b); err=best-8
        if err==0: err=turnpref*2
        log("TURN best=%d err=%d pref=%d ticks=%d"%(best,err,turnpref,tk))"""
src=src.replace(a3,b3)
open('/bot/src/explore5.py','w').write(src)
import ast; ast.parse(src); print("patched")
