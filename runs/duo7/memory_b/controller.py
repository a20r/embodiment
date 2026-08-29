import time, math, threading, sys
sys.path.insert(0,'/bot/src')
from lib import rd, wr, drive, stop, tx

LOG='/tmp/ctrl.log'
def log(msg):
    with open(LOG,'a') as f: f.write('%.1f %s\n'%(time.time(),msg))

state={'d5':None,'d2':'0','goal':0,'tick':0}
def d5_thread():
    while True:
        try:
            with open('/dev/robot/d5') as f:
                line=f.readline().strip()
            if line:
                state['d5']=line
                log('d5 '+line)
        except: time.sleep(0.5)
threading.Thread(target=d5_thread,daemon=True).start()

def lidar():
    while True:
        try:
            v=[float(x) for x in rd('d3').split(',')]
            if len(v)==16: return v
        except: pass
def heading():
    while True:
        try: return float(rd('d1'))
        except: pass
def poll_status():
    try:
        s=rd('d6')
        t=int(s.split('tick=')[1].split()[0])
        g=int(s.split('goal=')[1].split()[0])
        state['tick']=t; state['goal']=g
    except: pass
    try: state['d2']=rd('d2')
    except: pass

def clean(l, prev):
    return [ (l[i] if l[i]>=0 else (prev[i] if prev else 3.0)) for i in range(16) ]

# dead reckoning
x,y=0.0,0.0
K=0.0065  # units per (cmd unit * sec), approx from 100 -> 0.65/s
def mainloop():
    global x,y
    prev=None
    last_bcast=0
    last_log=0
    t_last=time.time()
    mode='follow'
    cmdl=cmdr=0
    while True:
        l=lidar(); h=heading()
        l=clean(l,prev); prev=l
        now=time.time(); dt=now-t_last; t_last=now
        # integrate previous command
        v=K*(cmdl+cmdr)/2.0
        hr=math.radians(h)
        x+=v*dt*math.cos(hr); y+=v*dt*math.sin(hr)
        front=min(l[15],l[0],l[1])
        right=min(l[3],l[4],l[5])
        left=min(l[11],l[12],l[13])
        poll_status()
        if state['goal']==1:
            stop(); log('GOAL REACHED x=%.2f y=%.2f'%(x,y)); tx('GOAL REACHED by A'); 
            time.sleep(1); continue
        # right wall follow
        if front<0.28:
            cmdl,cmdr=-45,45   # turn left in place
        else:
            err = right-0.35
            steer = max(-30,min(30, err*120))
            base = 70 if front>0.6 else 40
            cmdl = base+steer; cmdr = base-steer
        drive(int(cmdl),int(cmdr))
        if now-last_log>0.5:
            last_log=now
            log('pos %.2f %.2f h=%.1f f=%.2f r=%.2f l8=%.2f goal=%d d2=%s'%(x,y,h,front,right,l[8],state['goal'],state['d2']))
        if now-last_bcast>3:
            last_bcast=now
            tx('A pos %.2f %.2f h %.1f'%(x,y,h))
        time.sleep(0.08)
mainloop()
