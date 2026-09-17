import os, sys, time, select
# hold ports open and stream all lines for N seconds
ports = sys.argv[2:] or ["d0","d3","d4","d5","d6","d9","d10","d11"]
dur = float(sys.argv[1])
fds = {}
for p in ports:
    fds[os.open('/dev/robot/'+p, os.O_RDONLY|os.O_NONBLOCK)] = p
bufs = {fd: b"" for fd in fds}
end = time.time()+dur
counts = {p:0 for p in ports}
last = {}
while time.time() < end:
    r,_,_ = select.select(list(fds),[],[],0.1)
    for fd in r:
        try: chunk = os.read(fd, 4096)
        except BlockingIOError: continue
        if not chunk: continue
        bufs[fd] += chunk
        while b"\n" in bufs[fd]:
            line, bufs[fd] = bufs[fd].split(b"\n",1)
            p = fds[fd]; counts[p]+=1
            s = line.decode(errors='replace')
            if p in ("d3","d10") or counts[p] <= 5:
                print(f"{time.time():.2f} {p}: {s}")
            last[p]=s
print("counts:", counts)
print("last:", last)
