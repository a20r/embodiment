import os, time
def w(p, s):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY)
        os.write(fd, (s+'\n').encode())
        os.close(fd)
        print(f"write {p} <- {s!r}: ok", flush=True)
    except Exception as e:
        print(f"write {p} <- {s!r}: FAIL {type(e).__name__} {e}", flush=True)
w('d1','0')
time.sleep(0.5)
w('d7','0')
time.sleep(0.5)
w('d5','0')
time.sleep(0.5)
w('d6','0')
time.sleep(0.5)
w('d0','0')
