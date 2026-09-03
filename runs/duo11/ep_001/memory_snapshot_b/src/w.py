import os, sys, time
dev, val = sys.argv[1], sys.argv[2]
fd = os.open(dev, os.O_WRONLY | os.O_NONBLOCK)
os.write(fd, (val+"\n").encode())
os.close(fd)
print("wrote", val, "to", dev)
