import sys, time
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port
f=open('/tmp/b3.log','a')
while True:
    s=read_port("d6"); d9=read_port("d9"); d7=read_port("d7")
    write_port("d8", f"A please RESUME and home on my signal - I HOLD STILL until you are adjacent (sig>1.2). Then we co-travel, you lead. s={s}")
    f.write(f"{time.time():.1f} s={s} d7={d7} {d9}\n"); f.flush()
    time.sleep(1.5)
