import time,sys
sys.path.insert(0,"/bot/src")
from robot import *
L=open("/bot/src/d5test.log","a",buffering=1)
def log(s): L.write("%.1f %s\n"%(time.time(),s))
def d5():
    v=rd("d5",timeout=0.2)
    return v
def enc_avg():
    a=rd("d9"); b=rd("d6")
    try: return (float(a)+float(b))/2
    except: return None
log("START idle")
e0=enc_avg(); t0=time.time(); on=0; off=0
while time.time()-t0<6:
    v=d5()
    if v=="1": on+=1
    else: off+=1
    time.sleep(0.1)
log("IDLE on=%d off=%d enc=%s"%(on,off,enc_avg()))
# drive forward
t0=time.time(); on=0; off=0; e1=enc_avg()
motors(9,9)
while time.time()-t0<6:
    v=d5()
    if v=="1": on+=1
    else: off+=1
    time.sleep(0.1)
motors(0,0)
log("FWD on=%d off=%d enc=%s->%s"%(on,off,e1,enc_avg()))
time.sleep(2)
# rotate in place
t0=time.time(); on=0; off=0
motors(5,-5)
while time.time()-t0<5:
    v=d5()
    if v=="1": on+=1
    else: off+=1
    time.sleep(0.1)
motors(0,0)
log("ROT on=%d off=%d"%(on,off))
log("END")
