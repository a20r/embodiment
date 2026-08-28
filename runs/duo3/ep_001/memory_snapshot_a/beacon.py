import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port
LOG=open('/tmp/beacon.log','a')
i=0
while True:
    s=read_port("d6"); d9=read_port("d9")
    write_port("d8", f"B holding s={s} come to me")
    LOG.write(f"{time.time():.1f} s={s} {d9}\n"); LOG.flush()
    time.sleep(1)
