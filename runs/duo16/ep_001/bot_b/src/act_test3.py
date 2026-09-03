import os, time, signal
base='/dev/robot/'
def read_port(name, timeout=0.4):
    def handler(s,f): raise TimeoutError()
    signal.signal(signal.SIGALRM, handler)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    try:
        fd = os.open(base+name, os.O_RDONLY)
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

def scan(n=1):
    out=[]
    for i in range(n):
        s = read_port('d2',0.6)
        out.append(s.decode())
    return out

def sample(label, n=6):
    for i in range(n):
        vals = {k: read_port(k,0.3) for k in ['d0','d3','d4','d5','d6','d9','d11']}
        print(label, {k: v[:20] for k,v in vals.items()})
        time.sleep(0.12)

print('scan before:'); print(scan()[0])
sample('pre:',3)
# spin test: d1=2, d7=-2
write_port('d1','2\n'); write_port('d7','-2\n')
sample('spin:',6)
write_port('d1','0\n'); write_port('d7','0\n')
sample('stop:',3)
print('scan after spin:'); print(scan()[0])
