import os, select, time, threading
def try_write(p, s='0'):
    try:
        fd = os.open(f'/dev/robot/{p}', os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, (s+'\n').encode()); os.close(fd)
        return "ok"
    except Exception as e:
        return f"{type(e).__name__}"
for p in ['d0','d4','d5','d6','d9','d10','d11']:
    print(p, "write:", try_write(p), flush=True)

# poll d10 for 12s while saying hello every 2s
stop=[False]
def hello():
    t0=time.time()
    while not stop[0]:
        try:
            fd=os.open('/dev/robot/d8', os.O_WRONLY); os.write(fd,b'HELLO ARE YOU THERE?\n'); os.close(fd)
        except Exception as e: print("tx err",e)
        time.sleep(2.0)
threading.Thread(target=hello,daemon=True).start()
t0=time.time(); n=0
while time.time()-t0<12:
    fd = os.open('/dev/robot/d10', os.O_RDONLY)
    r,_,_ = select.select([fd],[],[],0.5)
    if r:
        d=os.read(fd,4096); n+=1
        if d.strip(): print(f"RX: {d!r}", flush=True)
    os.close(fd)
stop[0]=True
print("d10 polls with data:", n)
