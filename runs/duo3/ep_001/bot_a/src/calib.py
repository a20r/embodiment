import sys, time, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass
from drive import motors, stop, turn_by

def front_avg(n=5):
    vs=[]
    for _ in range(n):
        l=lidar()
        if l and l[0]>0: vs.append(l[0])
    return sum(vs)/len(vs) if vs else None

def face_open():
    L=lidar()
    i=max(range(16), key=lambda k:(3 if L[k]<0 else L[k]))
    turn_by(((22.5*i+180)%360)-180)
    time.sleep(0.2)

res={}
for cmd in [30,40,50,60,80,100,140,200]:
    f0=front_avg()
    if f0 is None or f0<1.0:
        face_open(); f0=front_avg()
    if f0 is None or f0<0.9:
        # back up to make room
        motors(-80,-80); time.sleep(1.0); stop(); face_open(); f0=front_avg()
    motors(cmd,cmd); time.sleep(1.0); stop(); time.sleep(0.3)
    f1=front_avg()
    res[cmd]=round(f0-f1,3)
    print(cmd, round(f0,3), round(f1,3), "speed", res[cmd], flush=True)
print(json.dumps(res))
