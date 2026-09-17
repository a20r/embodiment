import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, math, json
TPU=1850.0; PF='/bot/pose.json'
st=json.load(open(PF)); x,y=st['x'],st['y']
def d11(n=12):
    v=[]
    for _ in range(n):
        s=rio.read_line('d11',0.5)
        try: v.append(float(s))
        except: pass
        time.sleep(0.05)
    v.sort(); return sum(v[2:-2])/len(v[2:-2])
def dv(h): r=math.radians(h); return math.sin(r),math.cos(r)
pts=[(x,y,d11())]; print('P0',pts[-1])
for ang in [float(a) for a in sys.argv[1:]]:
    ctl.turn_to(ang,tol=4); l0,r0=ctl.enc()
    trav,reason=ctl.forward(0.3*TPU,speed=50,hold_heading=ang,min_front=0.2,timeout=8)
    l1,r1=ctl.enc(); dd=((l1-l0)+(r1-r0))/2/TPU; dx,dy=dv(ang); x+=dx*dd; y+=dy*dd
    time.sleep(0.3); pts.append((x,y,d11())); print('P',pts[-1],reason)
json.dump({**st,'x':x,'y':y},open(PF,'w'))
for k in [2.0,2.5]:
    best=None
    for bx in [x+i*0.05 for i in range(-40,41)]:
        for by in [y+i*0.05 for i in range(-40,41)]:
            err=sum((1/(1+(math.hypot(px-bx,py-by)/k)**2)-v)**2 for px,py,v in pts)
            if best is None or err<best[0]: best=(err,bx,by)
    print(f'k={k}: B at ({best[1]:.2f},{best[2]:.2f}) err={best[0]:.5f}; from me: dist={math.hypot(best[1]-x,best[2]-y):.2f} bearing={math.degrees(math.atan2(best[1]-x,best[2]-y))%360:.0f}')
h=ctl.heading(); s=ctl.scan(); print('hdg',h,'pose',round(x,2),round(y,2)); print(' '.join(f'{(h+22.5*k)%360:3.0f}:{s[k]:.2f}' for k in range(16)))
