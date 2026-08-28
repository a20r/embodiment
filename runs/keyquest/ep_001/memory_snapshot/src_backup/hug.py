import sys, time, math, json
sys.path.insert(0,'/bot/src')
from lib import *
LOG=open('/bot/followL.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)
prev=[0.5]*16
def lid():
    global prev
    v=lidar()
    if v is None: return prev
    v=[prev[i] if v[i]<0 else v[i] for i in range(16)]
    prev=v; return v
step=0
t0=time.time()
smax=0
while True:
    step+=1
    if goal():
        drive(0,0); log('GOAL!')
        open('/memory/GOAL.txt','a').write('GOAL by followL\n')
        break
    v=lid()
    front=min(v[15],v[0],v[1])
    left=min(v[3],v[4])
    lfront=v[2]
    s=rdf(4)
    if s and step%8==0:
        smax=max(smax,s)
        log(f'step{step} s={s} smax={smax} front={front:.2f} left={left:.2f}')
    if front<0.14:
        drive(5,-5); time.sleep(0.25); continue  # turn right
    if left>0.45 and lfront>0.35:
        drive(3,6)   # arc left
    elif left<0.10:
        drive(6,4)
    elif left>0.22:
        drive(4,6)
    else:
        drive(6,6)
    time.sleep(0.25)
