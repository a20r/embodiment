from rob import *
import sys
a=sys.argv[1]; b=sys.argv[2]; dur=float(sys.argv[3])
def pr(tag):
    r=rd('d2'); print(tag, "d4=",rd('d4'),"d0=",rd('d0'),"d5=",rd('d5'),"d3=",rd('d3')); print("   r:",r)
pr("before")
wr('d1',a); wr('d7',b)
t0=time.time()
while time.time()-t0<dur:
    time.sleep(0.5)
    print(f"t={time.time()-t0:.1f} d4={rd('d4',0.3)} d0={rd('d0',0.3)} d5={rd('d5',0.3)} d6={rd('d6',0.3)} d9={rd('d9',0.3)}")
wr('d1','0'); wr('d7','0')
time.sleep(0.5)
pr("after")
