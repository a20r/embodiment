import rio, time, math, statistics, sys
def stop(): rio.wr('d7','0'); rio.wr('d1','0')
def enc():
    try: return int(rio.rd('d6')), int(rio.rd('d9'))
    except: return enc()
def heading(n=3):
    xs=[];ys=[]
    for _ in range(n):
        try: h=math.radians(float(rio.rd('d4')))
        except: continue
        xs.append(math.cos(h)); ys.append(math.sin(h))
    return math.degrees(math.atan2(sum(ys),sum(xs)))%360
def scan(n=3):
    ss=[]
    for _ in range(n):
        s=rio.rd('d2',0.3)
        if s:
            try: ss.append([float(v) for v in s.split(',')])
            except: pass
    if not ss: return [-1]*16
    return [statistics.median([s[i] for s in ss if s[i]>=0] or [-1]) for i in range(16)]
def front(sc):
    c=[sc[0] if sc[0]>=0 else 9]
    for k in (1,15):
        if 0<=sc[k]<0.32: c.append(sc[k]*0.92)
    return min(c)
def status():
    return 'L=%d R=%d h=%.0f d11=%s %s'%(*enc(), heading(), rio.rd('d11'), rio.rd('d3'))
def angdiff(a,b): return (a-b+180)%360-180
def turn(deg, v=20):
    """rotate CCW by deg (neg=CW), closed loop on compass"""
    h0=heading(5); target=(h0+deg)%360
    t0=time.time()
    while time.time()-t0<40:
        err=angdiff(target, heading(3))
        if abs(err)<3: break
        sp=max(6,min(v, abs(err)*0.6))
        s=1 if err>0 else -1
        rio.wr('d1',str(s*sp)); rio.wr('d7',str(-s*sp))
        time.sleep(0.1)
    stop(); time.sleep(0.3)
    return heading(5)
def forward(dist, v=40, stopfront=0.13, hold=True):
    """drive forward dist enc units (avg of wheels) w/ heading hold; stop early if front blocked"""
    L0,R0=enc(); h0=heading(5); t0=time.time(); reason='dist'
    while True:
        L,R=enc(); d=((L-L0)+(R-R0))/2
        if d>=dist: break
        if time.time()-t0>dist/(5*v)+15: reason='timeout'; break
        sc=scan(1)
        if front(sc)<stopfront and front(sc)>=0: reason='front=%.2f'%front(sc); break
        corr=0
        if hold: corr=max(-0.3,min(0.3,angdiff(h0,heading(1))*0.03))  # err>0 => need CCW => right faster
        rio.wr('d7',str(v*(1-corr))); rio.wr('d1',str(v*(1+corr)*1.05))
        time.sleep(0.1)
    stop(); time.sleep(0.3)
    L,R=enc()
    return ((L-L0)+(R-R0))/2, reason
if __name__=='__main__':
    cmd=sys.argv[1]
    if cmd=='turn': print(turn(float(sys.argv[2]))); print(status())
    elif cmd=='fwd': print(forward(float(sys.argv[2]), float(sys.argv[3]) if len(sys.argv)>3 else 40)); print(status())
    elif cmd=='scan': sc=scan(5); print(' '.join('%d:%.2f'%(i,v) for i,v in enumerate(sc))); print(status())
    elif cmd=='stop': stop()
