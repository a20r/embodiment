import os, select, time
D='/dev/robot/'
def read(p, timeout=0.2):
    fd=os.open(D+p, os.O_RDONLY|os.O_NONBLOCK); r,_,_=select.select([fd],[],[],timeout)
    out=''
    if r:
        try: out=os.read(fd,200000).decode().strip()
        except: out=''
    os.close(fd); return out
def w(p,msg):
    try:
        fd=os.open(D+p,os.O_WRONLY|os.O_NONBLOCK); os.write(fd,msg.encode()); os.close(fd)
    except Exception: pass

print("--- 6s with NO transmit")
for i in range(12):
    print(read('d5'), read('d11'), end='; ')
    time.sleep(0.5)
print()
print("--- transmit bursts, watch d5/d10")
for k in range(3):
    w('d8', "TEST MSG 123\n")
    time.sleep(0.1)
    print("after TX:", read('d5'), read('d10'), read('d11'))
    time.sleep(1.0)
