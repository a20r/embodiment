import os, select

for i in range(12):
    p = f"/dev/robot/d{i}"
    try:
        fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
        r,_,_ = select.select([fd], [], [], 0.5)
        if r:
            data = os.read(fd, 4096)
            print(f"d{i} READ: {data!r}")
        else:
            print(f"d{i} read: no data (timeout)")
        os.close(fd)
    except Exception as e:
        print(f"d{i} read error: {e}")
