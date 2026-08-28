import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port
f=open('/tmp/tri.log','a')
t0=time.time()
while time.time()-t0<float(sys.argv[1]):
    s=read_port("d6")
    if s: f.write(f"{time.time():.2f} {s}\n"); f.flush()
    write_port("d8","T mapping you, keep moving")
    time.sleep(0.4)
