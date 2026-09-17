import os, select
for i in range(12):
    p = f"/dev/robot/d{i}"
    res = {}
    # try read
    try:
        fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
        r,_,_ = select.select([fd],[],[],0.5)
        if r:
            try:
                data = os.read(fd, 256)
                res['read'] = repr(data[:120])
            except OSError as e:
                res['read'] = f"err {e}"
        else:
            res['read'] = "no data (open ok, empty)"
        os.close(fd)
    except OSError as e:
        res['read'] = f"open fail: {e}"
    # try write
    try:
        fd = os.open(p, os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, b"?\n")
        res['write'] = "ok"
        os.close(fd)
    except OSError as e:
        res['write'] = f"fail: {e}"
    print(f"d{i}: {res}")
