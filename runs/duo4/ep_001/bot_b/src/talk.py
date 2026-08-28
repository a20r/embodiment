import os,time
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx=os.open(DEV+"d3",os.O_RDONLY|os.O_NONBLOCK); buf=b""
log=open("/memory/run.log","a",buffering=1)
msgs=["PONG alpha","PING beta","HELLO alpha","alpha","STATUS","WHERE","GOAL?","FOLLOW ME","HELP"]
i=0; t0=time.time(); lasttx=0
while time.time()-t0<50:
    try:
        c=os.read(rx,4096); buf+=c
    except BlockingIOError: pass
    while b"\n" in buf:
        l,buf=buf.split(b"\n",1)
        t=l.decode(errors="replace").strip()
        if t: log.write(f"RX2 {time.time()-t0:.1f} {t!r}\n"); print("RX",t)
    if time.time()-lasttx>4:
        m=msgs[i%len(msgs)]; i+=1
        wr("d8",m); log.write(f"TX2 {time.time()-t0:.1f} {m!r}\n"); print("TX",m)
        lasttx=time.time()
    time.sleep(0.05)
