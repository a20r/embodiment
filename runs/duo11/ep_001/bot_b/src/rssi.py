import os,time
def sample(dur=1.2):
    fd=os.open("/dev/robot/d11",os.O_RDONLY|os.O_NONBLOCK)
    buf=b""; t0=time.time()
    while time.time()-t0<dur:
        try:
            d=os.read(fd,4096)
            if d: buf+=d
            else: time.sleep(0.01)
        except BlockingIOError: time.sleep(0.01)
    os.close(fd)
    vals=[]
    for l in buf.split(b"\n"):
        try: vals.append(float(l))
        except: pass
    return sum(vals)/max(1,len(vals))
if __name__=="__main__":
    print(f"{sample():.4f}")
