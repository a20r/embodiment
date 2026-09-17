import os, time, signal
base='/dev/robot/'
def read_port(name, timeout=1.0):
    def handler(s,f): raise TimeoutError()
    signal.signal(signal.SIGALRM, handler)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        fd = os.open(base+name, os.O_RDONLY)
        signal.setitimer(signal.ITIMER_REAL, timeout)
        data = os.read(fd, 8000)
        os.close(fd)
        signal.alarm(0)
        return data
    except TimeoutError:
        return b'<TIMEOUT>'
    finally:
        signal.alarm(0)

for name in ['d0','d1','d2','d3','d4','d5','d6','d7','d8','d9','d10','d11']:
    d = read_port(name)
    print(name, repr(d[:150]))
