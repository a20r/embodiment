import os, select, time
D='/dev/robot/'
def read(p, timeout=0.25):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,200000).decode().strip()
        except: out=''
    os.close(fd); return out

for i in range(10):
    print(read('d3'), "| d0:", read('d0'), "d5:", read('d5'), "d11:", read('d11'), "head:", read('d4'))
    time.sleep(0.5)
