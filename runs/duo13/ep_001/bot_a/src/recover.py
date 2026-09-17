import sys, time; sys.path.insert(0,'/bot/src')
from robot import R
r=R()
r.motors(0,0); time.sleep(0.2)
h=r.heading(); print("h=",h,"front=",(r.ranges() or [9])[0],"st=",r.status(),"enc=",r.enc())
r.motors(-70,-70); time.sleep(0.8); r.motors(0,0); time.sleep(0.3)
print("backed: front=",(r.ranges() or [9])[0],"h=",r.heading(),"enc=",r.enc())
r.write(8,"PING1 anybot?")
time.sleep(0.3); print("rx:",repr(r.read(10,0.5)))
