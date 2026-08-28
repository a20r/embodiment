import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port
f=open('/tmp/b4.log','a')
while True:
    s=read_port("d6"); d9=read_port("d9"); d7=read_port("d7")
    write_port("d8", f"B HOLDING STILL as agreed. my sig of you s={s}. come adjacent; then you lead, I tail you.")
    f.write(f"{time.time():.1f} s={s} d7={d7} {d9}\n"); f.flush()
    time.sleep(1.5)
