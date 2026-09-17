import sys, time, threading
sys.path.insert(0,'/bot/src')
from rob import *
import threading

stop = threading.Event()
def rx_loop():
    while not stop.is_set():
        d = read_port('d10', 0.5)
        if d and d.strip():
            print('RX:', d[:120], flush=True)
t = threading.Thread(target=rx_loop, daemon=True); t.start()

def stats():
    pts = scan()
    if not pts: return None
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; zs=[p[2] for p in pts]
    return 'n=%d x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (len(pts),min(xs),max(xs),min(ys),max(ys),min(zs),max(zs))

write_port('d8','hello from A')
print('sent hello')
print('pre:', stats(), read_line('d4'), read_line('d11'), read_line('d0'), read_line('d5'))
write_port('d1','6'); write_port('d7','-6')
for i in range(10):
    time.sleep(0.5)
    print(i, stats(), 'd4=', read_line('d4'), 'd11=', read_line('d11'), flush=True)
write_port('d1','0'); write_port('d7','0')
time.sleep(1.5)
print('post:', stats(), 'd4=', read_line('d4'), 'd11=', read_line('d11'))
time.sleep(2)
print('rest:', stats(), 'd4=', read_line('d4'), 'd11=', read_line('d11'))
stop.set()
