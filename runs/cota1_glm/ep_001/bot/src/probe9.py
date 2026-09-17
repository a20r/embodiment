import time, threading
from rd import rd, wr
stop=False
def pump(p,v,hz=20):
    while not stop:
        try: wr(p,v)
        except Exception: pass
        time.sleep(1.0/hz)
t0=time.time()
th=threading.Thread(target=pump,args=('d4','0.6')); th.start()
for i in range(30):
    print(f'{time.time()-t0:5.1f} d0={rd("d0",0.1)} d2={rd("d2",0.1)} d3={rd("d3",0.1)}')
    print('    d5',rd('d5',0.1))
    time.sleep(0.1)
stop=True; th.join()
print('--- released ---')
for i in range(10):
    print(f'{time.time()-t0:5.1f} d0={rd("d0",0.1)} d5={rd("d5",0.1)}')
    time.sleep(0.2)
