import os, sys

for name in sorted(os.listdir('/dev/robot')):
    p = '/dev/robot/' + name
    # try read
    try:
        fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
        try:
            data = os.read(fd, 4096)
            print(f"{name}: READ -> {data[:200]!r}")
        except BlockingIOError:
            print(f"{name}: read OK (would block, empty)")
        os.close(fd)
    except Exception as e:
        print(f"{name}: read FAIL {type(e).__name__} {e}")
    # try write
    try:
        fd = os.open(p, os.O_WRONLY | os.O_NONBLOCK)
        try:
            n = os.write(fd, b"ping\n")
            print(f"{name}: WRITE accepted ({n} bytes)")
        except BlockingIOError:
            print(f"{name}: write OK (would block)")
        os.close(fd)
    except Exception as e:
        print(f"{name}: write FAIL {type(e).__name__} {e}")
