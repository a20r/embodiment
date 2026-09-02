import os, time, sys, threading
# continuously read d2 (lidar) and d4 (compass), print latest once per 0.5s
def reader(dev, store, key):
    fd = os.open(dev, os.O_RDONLY|os.O_NONBLOCK)
    buf=b""
    while True:
        try:
            d=os.read(fd,4096)
            if d:
                buf+=d
                lines=buf.split(b"\n")
                buf=lines[-1]
                for l in lines[:-1]:
                    if l.strip(): store[key]=l.decode()
            else: time.sleep(0.02)
        except BlockingIOError: time.sleep(0.02)
store={}
threading.Thread(target=reader,args=("/dev/robot/d2",store,"lidar"),daemon=True).start()
threading.Thread(target=reader,args=("/dev/robot/d4",store,"hdg"),daemon=True).start()
threading.Thread(target=reader,args=("/dev/robot/d3",store,"stat"),daemon=True).start()
dur=float(sys.argv[1])
end=time.time()+dur
while time.time()<end:
    print(store.get("hdg"), "|", store.get("stat"), "|", store.get("lidar"))
    time.sleep(0.5)
