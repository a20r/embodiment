import os, time, threading

def reader(path, out):
    f = open(path, 'r')
    while True:
        line = f.readline()
        if not line: break
        out.append((time.time(), path, line.strip()))

buf = []
for d in range(5):
    threading.Thread(target=reader, args=(f'/dev/robot/d{d}', buf), daemon=True).start()
time.sleep(3)
for t,p,l in buf[-40:]:
    print(f"{t%100:6.2f} {p[-2:]} {l}")
print("count per port:", {p: sum(1 for _,pp,_ in buf if pp==p) for p in set(x[1] for x in buf)})
