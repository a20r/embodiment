import os, time
fds={}
for n in [0,1,2,3,4,5,8]:
    fds[n]=os.open(f"/dev/robot/d{n}", os.O_RDONLY|os.O_NONBLOCK)
buf={n:b"" for n in fds}
last={n:None for n in fds}
t0=time.time()
while time.time()-t0<20:
    for n,fd in fds.items():
        try:
            while True:
                d=os.read(fd,65536)
                if not d: break
                buf[n]+=d
        except BlockingIOError: pass
        if b"\n" in buf[n]:
            ls=buf[n].split(b"\n"); buf[n]=ls[-1]; last[n]=ls[-2].decode()[:40]
    print(round(time.time()-t0,1), last[4], "|", last[2], last[8], "|", (last[1] or "")[:20])
    time.sleep(1)
