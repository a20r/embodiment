import os, time, signal, threading
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

def scan_stats():
    s = read_port('d2', 0.8)
    pts=[p.split(',') for p in s.decode().strip().split(';') if p]
    if not pts: return None
    xs=[float(p[0]) for p in pts]; ys=[float(p[1]) for p in pts]; zs=[float(p[2]) for p in pts]
    return dict(n=len(pts), x0=min(xs), x1=max(xs), y1=max(ys), z0=min(zs), z1=max(zs))

def rx_loop(stop):
    while not stop.is_set():
        d = read_port('d10', 0.5)
        if d and d.strip():
            print('RX:', d[:100])

stop = threading.Event()
t = threading.Thread(target=rx_loop, args=(stop,), daemon=True)
t.start()

write_port('d8','hello from A\n')
print('sent hello')
print('pre-spin scan:', scan_stats(), 'd4=', read_port('d4'), 'd11=', read_port('d11'), 'd0=', read_port('d0'), 'd5=', read_port('d5'))
# spin
write_port('d1','6\n'); write_port('d7','-6\n')
for i in range(12):
    time.sleep(0.4)
    st = scan_stats()
    print(i, st, 'd4=', read_port('d4')[:8], 'd11=', read_port('d11')[:8])
write_port('d1','0\n'); write_port('d7','0\n')
print('post-spin scan:', scan_stats(), 'd4=', read_port('d4'))
time.sleep(2)
print('after stop scan:', scan_stats(), 'd4=', read_port('d4'), 'd11=', read_port('d11'))
stop.set()
