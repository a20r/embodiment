import os, time

def read(p, timeout=0.2):
    fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = None
    if r:
        try: out = os.read(fd, 4096)
        except Exception as e: out = b''
    os.close(fd)
    return out

import select

def w(p, msg):
    try:
        fd = os.open(p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, msg if isinstance(msg, bytes) else msg.encode())
        os.close(fd)
        return True
    except Exception as e:
        print("write err", p, e); return False

# baseline
print("baseline:", read('/dev/robot/d3'), read('/dev/robot/d4'), read('/dev/robot/d9'))
# write to d1 only
w('/dev/robot/d1', "50\n")
for i in range(6):
    time.sleep(0.4)
    print("d1=50:", read('/dev/robot/d3'), read('/dev/robot/d4'), read('/dev/robot/d9'))
w('/dev/robot/d1', "0\n")
time.sleep(0.5)
# write to d7 only
w('/dev/robot/d7', "50\n")
for i in range(6):
    time.sleep(0.4)
    print("d7=50:", read('/dev/robot/d3'), read('/dev/robot/d4'), read('/dev/robot/d9'))
w('/dev/robot/d7', "0\n")
