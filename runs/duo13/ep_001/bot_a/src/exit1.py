import sys, time; sys.path.insert(0,'/bot/src')
from drive import Driver, wrap
d=Driver(); r=d.r
r.motors(0,0); time.sleep(0.3)
print("start h=", r.heading(), "enc=", r.enc(), "st=", r.status())
d.turnto(88, tol=3)
time.sleep(0.3); print("turned to", r.heading())
d.fwdtarget = r.heading()
cl,cr = d.fwd(90, lambda: (r.ranges() or [9])[0] < 0.5, maxt=10)
print("after drive enc=", (cl,cr), "h=", r.heading(), "front=", (r.ranges() or [9])[0])
print("status=", r.status())
