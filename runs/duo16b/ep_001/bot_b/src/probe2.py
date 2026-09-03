import os, time, select

ports = ['d0','d1','d2','d3','d4','d5','d6','d7','d8','d9','d10','d11']
fds = {}
for name in ports:
    fds[name] = os.open('/dev/robot/'+name, os.O_RDONLY | os.O_NONBLOCK)

# Write to d1
try:
    os.write(fds['d1'], b"ping\n")
except Exception as e:
    print("d1 write err", e)
t0 = time.time()
while time.time() - t0 < 3:
    r,_,_ = select.select(list(fds.values()), [], [], 0.2)
    for fd in r:
        for name, f in fds.items():
            if f == fd:
                try:
                    data = os.read(fd, 4096)
                    print(f"[{time.time()-t0:.2f}s] {name}: {data[:300]!r}")
                except Exception as e:
                    print(name, "err", e)
print("--- now d7 ---")
try:
    os.write(fds['d7'], b"ping\n")
except Exception as e:
    print("d7 write err", e)
t0 = time.time()
while time.time() - t0 < 3:
    r,_,_ = select.select(list(fds.values()), [], [], 0.2)
    for fd in r:
        for name, f in fds.items():
            if f == fd:
                try:
                    data = os.read(fd, 4096)
                    print(f"[{time.time()-t0:.2f}s] {name}: {data[:300]!r}")
                except Exception as e:
                    print(name, "err", e)
print("--- done ---")
