import sys, time, math
sys.path.insert(0,'/bot/src')
from rob import *

def rl(name, timeout=0.5, tries=8):
    for i in range(tries):
        d = read_port(name, timeout).decode(errors='replace')
        lines=[l for l in d.strip().split('\n') if l.strip()]
        if lines:
            try: return float(lines[-1])
            except: return None
        time.sleep(0.05)
    return None
def drive(l,r,secs):
    try:
        write_port('d1',str(l)); write_port('d7',str(r)); time.sleep(secs)
        write_port('d1','0'); write_port('d7','0')
    except Exception as e:
        pass
log=open('/memory/nav.log','a')
def P(*a):
    s=' '.join(str(x) for x in a)
    try:
        log.write('%.1f %s\n'%(time.time()%10000, s)); log.flush()
    except: pass
    print(s, flush=True)

def turn_to(bearing, tol=5):
    for _ in range(10):
        h=rl('d4')
        if h is None: time.sleep(0.3); continue
        diff=(bearing-h+180)%360-180
        if abs(diff)<tol: return h
        t=min(abs(diff),40)/10.0
        if diff>0: drive(10,-10,t)
        else: drive(-10,10,t)
        time.sleep(0.25)
    return rl('d4')

def go(bearing_deg, max_secs=240, stop=0.12):
    t0=time.time()
    while time.time()-t0 < max_secs:
        turn_to(bearing_deg)
        o9=rl('d9'); o6=rl('d6'); d0=rl('d11')
        drive(25,25,1.2)
        o9b=rl('d9'); o6b=rl('d6'); d1=rl('d11')
        ticks=((o9b-o9)+(o6b-o6))/2 if None not in (o9,o6,o9b,o6b) else -1
        delta=(d1-d0) if None not in (d0,d1) else None
        st = read_line('d3',0.3)
        P('burst ticks=%s d11 %.4f->%.4f d=%s h=%s d3=%s'%(ticks,d0,d1,delta,rl('d4'),st))
        if d1 is not None and d1<stop:
            P('CLOSE d11=',d1); return d1
        if delta is not None and delta>0.015:
            P('d11 rising, stop'); return d1
    return None

if __name__=='__main__':
    bearing=float(sys.argv[1]) if len(sys.argv)>1 else 343.0
    try:
        r=go(bearing)
        P('NAV DONE result=',r,'d11=',rl('d11'),'d0=',rl('d0'),'d5=',rl('d5'))
    except Exception as e:
        P('NAV ERR',repr(e))
        drive(0,0,0.1)
