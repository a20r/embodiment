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
        return data
    except TimeoutError:
        return b'<TO>'
    finally:
        signal.alarm(0)

for i in range(8):
    t=time.time()
    out=[]
    for n in ['d0','d3','d4','d5','d6','d9','d11']:
        out.append('%s=%r'%(n, read_port(n,0.3)[:40]))
    print('%.1f'%(t%1000), ' '.join(out))
