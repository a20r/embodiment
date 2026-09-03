import os, time, select

def rd(name, dur=0.5):
    fd = os.open('/dev/robot/'+name, os.O_RDONLY | os.O_NONBLOCK)
    out=[]
    t0=time.time()
    while time.time()-t0<dur:
        r,_,_=select.select([fd],[],[],0.05)
        if r:
            try:
                d=os.read(fd,4096).decode().strip()
                if d: out.append(d)
            except Exception: pass
    os.close(fd)
    return out

w1 = os.open('/dev/robot/d1', os.O_WRONLY | os.O_NONBLOCK)
w7 = os.open('/dev/robot/d7', os.O_WRONLY | os.O_NONBLOCK)

print("before:", rd('d4',0.3)[-3:], rd('d11',0.3)[-3:])
# try writing a small number to d1
try:
    os.write(w1, b"0.3\n"); print("wrote d1=0.3")
except Exception as e: print("d1 err", e)
time.sleep(1.0)
print("after d1=0.3, d4:", rd('d4',0.3)[-3:], "d11:", rd('d11',0.3)[-3:])
time.sleep(1.0)
# stop
try:
    os.write(w1, b"0\n"); print("wrote d1=0")
except Exception as e: print("d1 err", e)
try:
    os.write(w7, b"0\n"); print("wrote d7=0")
except Exception as e: print("d7 err", e)
time.sleep(0.5)
print("after stop d4:", rd('d4',0.3)[-3:])
