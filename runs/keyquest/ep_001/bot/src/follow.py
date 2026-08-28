import sys, time, math, json, os
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/follow.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)

prev=[0.5]*16
def lid():
    global prev
    v=lidar()
    if v is None: return prev
    v=[prev[i] if v[i]<0 else v[i] for i in range(16)]
    prev=v; return v
def enc():
    l=rdf(6); r=rdf(2)
    return (l,r) if l is not None and r is not None else None

# persistent pose
POSE='/memory/pose.json'
x,y=0.0,0.0
try:
    with open(POSE) as f: p=json.load(f); x,y=p['x'],p['y']
except Exception: pass
last=enc()
pf=open('/memory/points.csv','a',buffering=1)
t0=time.time(); step=0; lastsave=0
log('start pose',x,y)
while True:
    step+=1
    if goal():
        drive(0,0); log('GOAL! pose',x,y)
        with open('/memory/GOAL.txt','a') as f: f.write(f'GOAL by follow.py pose {x:.0f},{y:.0f}\n')
        break
    v=lid(); h=hdg()
    e=enc()
    if e and last:
        d=((e[0]-last[0])+(e[1]-last[1]))/2.0
        x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
    last=e
    s=rdf(4)
    if s is not None and step%3==0: pf.write(f'{x:.1f},{y:.1f},{s}\n')
    if time.time()-lastsave>5:
        with open(POSE,'w') as f: json.dump({'x':x,'y':y},f)
        lastsave=time.time()
    front=min(v[15],v[0],v[1])
    right=min(v[12],v[13])
    rfront=v[14]
    if front<0.22:
        # turn left in place until clear
        drive(-5,5); time.sleep(0.25)
        continue
    # wall follow right at ~0.25
    if right>0.55 and rfront>0.4:
        # right opening: arc right
        drive(6,2)
    elif right<0.16:
        drive(3,6)
    elif right>0.32:
        drive(6,3)
    else:
        drive(6,6)
    time.sleep(0.25)
