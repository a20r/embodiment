import os, time, select
def w(p, s):
    fd = os.open(p, os.O_WRONLY); os.write(fd, (s+"\n").encode()); os.close(fd)
def r(i, to=0.03):
    try: fd = os.open(f"/dev/robot/d{i}", os.O_RDONLY|os.O_NONBLOCK)
    except OSError: return "E"
    rr,_,_ = select.select([fd],[],[],to)
    d = os.read(fd,256).decode().strip() if rr else "-"
    os.close(fd); return d
def snap(tag):
    print(tag, "d4=",r(4), "d2=",r(2), flush=True)
snap("start")
w("/dev/robot/d1","0.5"); w("/dev/robot/d7","-0.5")
for k in range(5):
    time.sleep(0.5); snap(f"spin {k}:")
w("/dev/robot/d1","0"); w("/dev/robot/d7","0")
time.sleep(0.5); snap("stop:")
