from rob import *
import sys
port=sys.argv[1]; val=sys.argv[2]; dur=float(sys.argv[3])
print("before:", rd('d4'), rd('d0'), rd('d5'), rd('d3')); print(" r:", rd('d2'))
wr(port,val)
t0=time.time()
while time.time()-t0<dur:
    time.sleep(0.5)
    print(f"t={time.time()-t0:.1f} d4={rd('d4',0.3)} d0={rd('d0',0.3)} d5={rd('d5',0.3)} d6={rd('d6',0.3)} d9={rd('d9',0.3)} d11={rd('d11',0.3)}")
wr(port,'0')
time.sleep(0.5)
print("after:", rd('d4'), rd('d0'), rd('d5'), rd('d3')); print(" r:", rd('d2'))
