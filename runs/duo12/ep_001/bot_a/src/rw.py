import os, time, sys
def write(dev, line):
    fd = os.open(dev, os.O_WRONLY | os.O_NONBLOCK)
    os.write(fd, (line+"\n").encode())
    os.close(fd)
def readfor(dev, t=1.0):
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    out=b""; end=time.time()+t
    while time.time()<end:
        try:
            b=os.read(fd,4096)
            if b: out+=b
            else: time.sleep(0.02)
        except BlockingIOError: time.sleep(0.02)
    os.close(fd)
    return out.decode(errors="replace")
if __name__=="__main__":
    cmd=sys.argv[1]
    if cmd=="w": write(sys.argv[2], sys.argv[3])
    elif cmd=="r": print(readfor(sys.argv[2], float(sys.argv[3]) if len(sys.argv)>3 else 1.0))
