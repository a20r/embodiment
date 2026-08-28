from ctl import Ctl
import sys, time
b=Ctl(); time.sleep(0.2)
s=[x if x>0 else 0.05 for x in b.scan()]
h=b.heading()
print('h',h)
for i,v in enumerate(s):
    print(i, round((h+i*22.5)%360,1), v)
