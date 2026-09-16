import os, time, sys

# Try reading each port with timeout
for name in ['d0','d1','d2','d3','d4','d5','d6','d7','d9','d11']:
    path = f'/dev/robot/{name}'
    t0 = time.time()
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        # read whatever's there
        try:
            data = os.read(fd, 65536)
        except BlockingIOError:
            data = b'<empty>'
        os.close(fd)
        print(f"{name}: READ ok ({time.time()-t0:.1f}s): {data[:200]}")
    except Exception as e:
        print(f"{name}: READ fail ({time.time()-t0:.1f}s): {type(e).__name__} {e}")
