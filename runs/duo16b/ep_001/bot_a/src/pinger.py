import os, time
D='/dev/robot/'
while True:
    try:
        fd = os.open(D+'d8', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, b"PING\n"); os.close(fd)
    except Exception: pass
    time.sleep(2.0)
