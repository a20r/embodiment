import time, sys
sys.path.insert(0,"/bot/src")
from robot import *
motors(0,0)
t0=time.time()
e6=rd("d6"); e9=rd("d9")
while time.time()-t0<25:
    time.sleep(1.2)
    n6=rd("d6"); n9=rd("d9")
    print("t=%.1f h=%s d6=%s(%s) d9=%s(%s) d0=%s d5=%s d11=%s" % (time.time()-t0, rd("d4"), n6, (float(n6)-float(e6)) if (n6 and e6) else '?', n9, (float(n9)-float(e9)) if (n9 and e9) else '?', rd("d0"), rd("d5"), rd("d11")))
    e6,e9=n6,n9
