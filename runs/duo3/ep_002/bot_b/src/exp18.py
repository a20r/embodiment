from ctl import Ctl
from unstick import unstick, sc
import time
b=Ctl(); time.sleep(0.3)
print('before',[round(x,2) for x in sc(b)])
print('ok',unstick(b))
print('after',[round(x,2) for x in sc(b)], 'h',b.heading())
