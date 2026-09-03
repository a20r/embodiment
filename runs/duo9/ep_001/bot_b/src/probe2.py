import sys; sys.path.insert(0,"/bot/src")
from lib import *
import time
def snap():
    return dict(d1=read_port("d1"), d2=read_port("d2"), d5=read_port("d5"), d3=read_port("d3"))
print("baseline", snap(), flush=True)
# sustained write to d10 only
end=time.time()+3
while time.time()<end:
    write_port("d10","1.0"); time.sleep(0.05)
print("after d10=1", snap(), flush=True)
end=time.time()+3
while time.time()<end:
    write_port("d11","1.0"); time.sleep(0.05)
print("after d11=1", snap(), flush=True)
