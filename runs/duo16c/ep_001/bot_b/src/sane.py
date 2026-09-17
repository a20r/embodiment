import math, time
from scene import read_line as rl
D2R=math.pi/180
def wr(p,v):
    for _ in range(3):
        try:
            fd=open(f'/dev/robot/{p}','w'); fd.write(f'{v}\n'); fd.close(); return
        except Exception: time.sleep(0.02)
def wheels(a,b,dur,period=0.1):
    end=time.time()+dur
    while time.time()<end:
        wr('d1',str(int(a))); wr('d7',str(int(b))); time.sleep(period)
def enc(): return float(rl('d6',0.4) or 0), float(rl('d9',0.4) or 0)
def hd(): return float(rl('d4',0.5) or 0)

x=y=0.0; H=0.0; h=hd(); e6,e9=enc()
def sense():
    global x,y,H,h,e6,e9
    nh=hd(); H+=(nh-h+180)%360-180; h=nh
    n6,n9=enc(); d6=n6-e6; d9=n9-e9; e6,e9=n6,n9
    dC=(d9+d6)/2*0.001
    x+=dC*math.cos(H*D2R); y+=dC*math.sin(H*D2R)
print('t0 xy=(%.2f,%.2f) H=%.1f'%(x,y,H))
for i in range(9):
    wheels(25,25,1.3); sense()
print('fwd ~2.3m: xy=(%.2f,%.2f) H=%.1f'%(x,y,H))
for i in range(9):
    wheels(-25,-25,1.3); sense()
print('back: xy=(%.2f,%.2f) H=%.1f'%(x,y,H))
for i in range(14):
    wheels(15,-15,0.95); sense()
print('spin: xy=(%.2f,%.2f) H=%.1f'%(x,y,H))
wheels(0,0,0.2)
