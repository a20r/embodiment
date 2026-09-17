import os, sys, time
dev = sys.argv[1]
timeout = float(sys.argv[2]) if len(sys.argv)>2 else 1.0
try:
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
except OSError as e:
    print(dev, "open err", e); sys.exit()
end = time.time()+timeout
buf=b""
while time.time()<end:
    try:
        d = os.read(fd, 4096)
        if d: buf+=d
        else: time.sleep(0.05)
    except BlockingIOError:
        time.sleep(0.05)
os.close(fd)
print(dev, repr(buf[:1000]))
