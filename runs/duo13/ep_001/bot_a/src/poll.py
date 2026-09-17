import os, select, time
reads = [0,2,3,4,5,6,9,10,11]
def readport(i, to=0.05):
    try:
        fd = os.open(f"/dev/robot/d{i}", os.O_RDONLY|os.O_NONBLOCK)
    except OSError as e:
        return f"ERR{e}"
    r,_,_ = select.select([fd],[],[],to)
    if r:
        try: data = os.read(fd,256).decode().strip()
        except OSError: data="E"
    else: data="-"
    os.close(fd)
    return data
t0=time.time()
for k in range(10):
    vals=[]
    for i in reads:
        vals.append(f"d{i}={readport(i)}")
    print(f"t={time.time()-t0:.1f} " + " ".join(vals), flush=True)
    time.sleep(0.7)
