src=open('/bot/src/explore5.py').read()
a="""t0=time.time(); lastlog=0; th_t=None
consec_turns=0; last_turn_tick=0
turnpref=+1
sigs=[]"""
b="""t0=time.time(); lastlog=0; th_t=None
consec_turns=0; last_turn_tick=0
turnpref=+1
sigs=[]
orbit_acc=0.0; orbit_mode=0; orbit_t=0
px=py=pth=None"""
src=src.replace(a,b)
a2="""    if th_t is None: th_t=havg()
    dl=min(b[4],b[5]); dr=min(b[11],b[12])"""
b2="""    if th_t is None: th_t=havg()
    # orbit detection: integrate heading change while driving
    global orbit_acc, orbit_mode, orbit_t, px, py, pth
    h_now=havg()
    if pth is not None:
        orbit_acc+=abs(wrap(h_now-pth))
    pth=h_now
    if orbit_acc>350 and orbit_mode==0:
        orbit_mode=1; orbit_t=time.time(); orbit_acc=0
        log("ORBIT detected - cutting across")
    if orbit_mode==1:
        th_t=(havg()+40*turnpref)%360
        dh2=wrap(th_t-havg())
        lb=9+int(round(0.09*dh2)); rb=9-int(round(0.09*dh2))
        motors(max(4,min(12,lb)),max(4,min(12,rb)))
        if time.time()-orbit_t>3.5:
            orbit_mode=0; th_t=havg()
        time.sleep(0.25)
        continue
    dl=min(b[4],b[5]); dr=min(b[11],b[12])"""
src=src.replace(a2,b2)
a3="""        log("TURN best=%d err=%d pref=%d ticks=%d"%(best,err,turnpref,tk))"""
b3="""        log("TURN best=%d err=%d pref=%d ticks=%d"%(best,err,turnpref,tk))
        orbit_acc=0"""
src=src.replace(a3,b3)
open('/bot/src/explore5.py','w').write(src)
import ast; ast.parse(src); print("patched")
