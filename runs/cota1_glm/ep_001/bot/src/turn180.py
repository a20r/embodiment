import time, math, sys
from drv import rd, wr
def f(x,d=0.0):
    try: return float(x)
    except: return d
def beams():
    try: return [float(x) for x in rd('d5').split(',')]
    except: return None
def wr2(d4,d7):
    wr('d4',str(int(d4))); wr('d7',str(int(d7)))
target=math.pi*float(sys.argv[1]) if len(sys.argv)>1 else math.pi*0.97
rot=0.0; pt=time.time(); phase=1; start_rot=0.0
try:
    while True:
        b=beams()
        if b is None: continue
        bc=[6.0 if x<0 else x for x in b]
        wz=f(rd('d0').split(',')[2]); sp=f(rd('d2'))
        t=time.time(); dt=min(t-pt,0.25); pt=t
        rot+=wz*dt
        prog=math.degrees(rot-start_rot)
        print(f'phase={phase} prog={prog:6.1f} v={sp:5.2f} wz={wz:5.2f} f={min(bc[15],bc[0],bc[1]):.2f} r={min(bc[7],bc[8],bc[9]):.2f}', flush=True)
        if abs(rot-start_rot)>=target: break
        if phase==1:
            wr2(-45,-85)   # reverse arc: yaw positive (CCW)
            if min(bc[7],bc[8],bc[9])<0.42 or abs(sp)<0.005:
                phase=0; start_rot=rot; print('-> switch forward arc', flush=True)
        else:
            wr2(30,85)     # forward arc CCW
            if min(bc[15],bc[0],bc[1])<0.42 or abs(sp)<0.005:
                phase=1; start_rot=rot; print('-> switch reverse arc', flush=True)
        time.sleep(0.06)
finally:
    wr2(0,0)
    print('TURN DONE, total prog this phase=%.0f'%(prog), flush=True)
