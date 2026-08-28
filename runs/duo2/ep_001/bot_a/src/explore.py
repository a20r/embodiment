import time, math, random, os, sys
from robot import Robot

r=Robot()
MAXR=2.5
x,y=0.0,0.0
visits={}
track=open('/memory/track.log','a')
track.write(f"# run3 {time.time()}\n")
SPEED=0.45

def clean(s):
    return [MAXR if (v is None or v<0) else min(v,MAXR) for v in s]

def check_goal():
    g=r.goal()
    if g:
        r.stop()
        print("GOAL REACHED at pose",x,y,flush=True)
        track.write(f"GOAL {x:.2f} {y:.2f}\n"); track.flush()
        sys.exit(0)

def mark(cx,cy):
    c=(round(cx/0.5),round(cy/0.5)); visits[c]=visits.get(c,0)+1

def eff(s,i):
    # effective clearance in direction i considering neighbors
    return min(s[i], s[(i+1)%16]*1.8+0.15, s[(i-1)%16]*1.8+0.15)

msg_t=0
last_choice=None
while True:
    check_goal()
    h=r.heading(); s=r.scan_med(3)
    if not s or h is None: time.sleep(0.2); continue
    s=clean(s)
    cand=[]
    for i in range(16):
        e=eff(s,i)
        b=math.radians(h+22.5*i)
        cx,cy=x+min(e,1.5)*math.cos(b), y+min(e,1.5)*math.sin(b)
        nov=3.0/(1+visits.get((round(cx/0.5),round(cy/0.5)),0))
        sc=e*1.2+nov+random.uniform(0,0.3)
        if e<0.45: sc-=5
        cand.append((sc,i,e))
    cand.sort(reverse=True)
    sc,best,e=cand[0]
    if e<0.45:
        # boxed in: reverse a bit
        track.write(f"BOXED x={x:.2f} y={y:.2f} scan={','.join('%.2f'%v for v in s)}\n"); track.flush()
        r.v.write(-25); time.sleep(1.5); r.v.write(0)
        hh=r.heading() or h
        x-=0.6*math.cos(math.radians(hh)); y-=0.6*math.sin(math.radians(hh))
        continue
    tgt=(h+22.5*best)%360
    track.write(f"P {x:.2f} {y:.2f} h={h:.0f} tgt={tgt:.0f} e={e:.2f} scan={','.join('%.2f'%v for v in s)}\n"); track.flush()
    mark(x,y)
    r.turn_to(tgt)
    check_goal()
    t_last=time.time(); t0=t_last
    hist=[]
    while time.time()-t0<25:
        h2=r.heading()
        s2=r.scan()
        if not s2 or h2 is None: continue
        s2=clean(s2)
        front=min(s2[0], s2[1]*2.2, s2[15]*2.2)
        now=time.time()
        hist.append((now,s2[0],h2))
        if front<0.32: break
        old=[(t,f,hh) for t,f,hh in hist if now-t>2.0]
        if old and now-t0>2.5:
            _,f_old,h_old=old[-1]
            if abs(f_old-s2[0])<0.04 and s2[0]<2.2:
                track.write("STUCK-nomove\n"); r.v.write(-25); time.sleep(1.2); r.v.write(0); break
            derr=(h2-h_old+180)%360-180
            if abs(derr)>30:
                track.write(f"WEDGED dh={derr:.0f}\n"); r.v.write(-25); time.sleep(1.2); r.v.write(0); break
        v=30 if front>0.7 else 16
        r.v.write(v)
        dt=now-t_last; t_last=now
        x+=SPEED*dt*math.cos(math.radians(h2)); y+=SPEED*dt*math.sin(math.radians(h2))
        mark(x,y)
        steer=(s2[1]-s2[15])*8
        r.w.write(round(max(min(steer,15),-15),1))
        check_goal()
        time.sleep(0.12)
    r.v.write(0); r.w.write(0)
    if time.time()-msg_t>20:
        r.tx.write("hello"); msg_t=time.time()
        m=r.rx.last_line(0.3)
        if m: print("RX:",m,flush=True); track.write("RX "+m+"\n")
