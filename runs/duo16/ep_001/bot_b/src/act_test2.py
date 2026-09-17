import os, time, signal
base='/dev/robot/'
def read_port(name, timeout=0.5):
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
def write_port(name, line):
    fd = os.open(base+name, os.O_WRONLY)
    os.write(fd, line.encode())
    os.close(fd)

def sample(label, n=8):
    for i in range(n):
        vals = {k: read_port(k,0.3) for k in ['d0','d3','d4','d5','d6','d9','d11']}
        print(label, {k: v[:20] for k,v in vals.items()})
        time.sleep(0.15)

write_port('d7','1\n')
print('wrote d7=1')
sample('d7=1:', 8)
write_port('d7','0\n')
print('wrote d7=0')
sample('d7=0:', 4)
write_port('d1','1\n')
time.sleep(0.1)
sample('both1:', 6)
write_port('d1','0\n'); write_port('d7','0\n')
sample('both0:', 3)
