import sys; sys.path.insert(0,'/bot/src')
from lib import *
import os, time

def d11(n=5):
    vs=[]
    for _ in range(n):
        v=last_of('d11',0.1)
        try: vs.append(float(v))
        except: pass
    return sum(vs)/len(vs) if vs else None

def fwd(sec,cmd):
    f1=os.open('/dev/robot/d1',os.O_WRONLY|os.O_NONBLOCK); os.write(f1,str(cmd).encode()+b"\n"); os.close(f1)
    f2=os.open('/dev/robot/d7',os.O_WRONLY|os.O_NONBLOCK); os.write(f2,str(cmd).encode()+b"\n"); os.close(f2)
    time.sleep(sec)
    stop(); time.sleep(0.2)

def rot(deg,cmd=55):
    h0=float(last_of('d4',0.15))
    tgt=h0+deg
    port='d1' if deg>0 else 'd7'
    fd=os.open('/dev/robot/'+port,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,str(cmd).encode()+b"\n"); os.close(fd)
    t0=time.time()
    while time.time()-t0<abs(deg)/(0.95*cmd)+1.2:
        h=last_of('d4',0.1)
        try: 
            if abs((float(h)-tgt+540)%360-180)<4: break
        except: pass
    stop(); time.sleep(0.15)
    return last_of('d4',0.15)

def check():
    s=last_of('d3',0.2)
    if 'goal=1' in s or 'here=1' in s: return True
    return False
