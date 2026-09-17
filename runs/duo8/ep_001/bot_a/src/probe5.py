import time
from lib import *
def fwd_test(v,t=2):
    l0=lidar()[0]; h0=heading(); t0=time.time()
    while time.time()-t0<t:
        motors(v,v); time.sleep(0.05)
    motors(0,0); time.sleep(0.2)
    l1=lidar()[0]
    print(f"v={v} beam0 {l0:.2f}->{l1:.2f} rate={(l0-l1)/t:.3f}/s h {h0}->{heading()}")
# back up first: negative?
fwd_test(-30,2)
fwd_test(50,1.5)
fwd_test(-50,1.5)
fwd_test(100,1.5)
print("others:", rline(2), rline(5), rline(7), rline(8), rline(9))
