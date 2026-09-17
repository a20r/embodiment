import os, time, signal, sys

class TO(Exception): pass
def handler(s,f): raise TO()
signal.signal(signal.SIGALRM, handler)

for name in ['d0','d1','d2','d3','d4','d5','d6','d7','d9','d11']:
    path = f'/dev/robot/{name}'
    t0 = time.time()
    try:
        fd = os.open(path, os.O_RDONLY)
        signal.alarm(3)
        try:
            data = os.read(fd, 200)
            signal.alarm(0)
            print(f"{name}: READ ok ({time.time()-t0:.1f}s): {data[:150]}")
        except TO:
            signal.alarm(0)
            print(f"{name}: READ blocked/no-data in 3s")
        os.close(fd)
    except Exception as e:
        print(f"{name}: READ fail ({time.time()-t0:.1f}s): {type(e).__name__} {e}")
