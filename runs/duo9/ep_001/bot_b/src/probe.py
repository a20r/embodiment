import threading, time, sys

def reader(path, out, stop):
    with open(path) as f:
        while not stop.is_set():
            line = f.readline().strip()
            out.append((time.time(), line))

stop = threading.Event()
d1, d2, d5 = [], [], []
for p, o in (("d1",d1),("d2",d2),("d5",d5)):
    threading.Thread(target=reader, args=(f"/dev/robot/{p}", o, stop), daemon=True).start()

time.sleep(1)
tests = ["1", "100", "f 1", "forward", "0.5 0.5", "v 1", "1,1"]
w = open("/dev/robot/d10","w", buffering=1)
w2 = open("/dev/robot/d11","w", buffering=1)
for t in tests:
    print("WRITE", t, flush=True)
    for i in range(10):
        w.write(t+"\n"); w2.write(t+"\n")
        time.sleep(0.1)
    print("d1:", [x[1] for x in d1[-3:]], "d2:", [x[1] for x in d2[-3:]], "d5:", [x[1] for x in d5[-3:]], flush=True)
stop.set()
