from rob import *
import datetime
f=open('/bot/src/rx.txt','a')
last=None
while True:
    m=rd('d10',1.0)
    if m:
        f.write(f"{datetime.datetime.now().strftime('%H:%M:%S')} RX: {m}\n"); f.flush()
    time.sleep(0.2)
