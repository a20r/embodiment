import os, time, signal, statistics
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
def f(v):
    try: return float(v)
    except: return None
def avg(name, n=5, dt=0.1):
    vals=[]
    for i in range(n):
        vals.append(f(read_port(name,0.3)))
        time.sleep(dt)
    vals=[v for v in vals if v is not None]
    return statistics.mean(vals) if vals else None

for cmd in ['5','-5','20','-20']:
    write_port('d1', cmd+'\n')
    time.sleep(0.3)
    v9 = avg('d9'); v4 = avg('d4'); v11 = avg('d11')
    print('d1=%s -> d9~%.1f d4~%.1f d11~%.3f' % (cmd, v9, v4, v11))
    write_port('d1','0\n')
    time.sleep(0.3)
print('--- d7 tests ---')
for cmd in ['5','-5','20','-20']:
    write_port('d7', cmd+'\n')
    time.sleep(0.3)
    v6 = avg('d6'); v4 = avg('d4')
    print('d7=%s -> d6~%.1f d4~%.1f' % (cmd, v6, v4))
    write_port('d7','0\n')
    time.sleep(0.3)
