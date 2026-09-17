import os, time
def w(p, s):
    fd = os.open(p, os.O_WRONLY)
    os.write(fd, (s+"\n").encode()); os.close(fd)
def r(i, to=0.05):
    try: fd = os.open(f"/dev/robot/d{i}", os.O_RDONLY|os.O_NONBLOCK)
    except OSError as e: return f"E"
    import select
    rr,_,_ = select.select([fd],[],[],to)
    d = os.read(fd,256).decode().strip() if rr else "-"
    os.close(fd); return d
print("baseline d4:", r(4), "d2:", r(2)[:60])
w("/dev/robot/d1","0.3")
time.sleep(1.0)
print("after d1=0.3: d4:", r(4), "d2:", r(2)[:60], "d0:",r(0),"d5:",r(5),"d6:",r(6),"d9:",r(9))
time.sleep(1.0)
w("/dev/robot/d1","0")
time.sleep(0.5)
print("after stop: d4:", r(4), "d2:", r(2)[:60])
