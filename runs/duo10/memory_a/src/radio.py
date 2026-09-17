import time, select, sys
D='/dev/robot/'
out=open('/bot/src/radio.log','a')
f=open(D+'d4')
while True:
    r,_,_=select.select([f],[],[],5)
    if r:
        line=f.readline().strip()
        if line:
            print(time.strftime('%H:%M:%S'), line, file=out); out.flush()
    else:
        time.sleep(0.1)
