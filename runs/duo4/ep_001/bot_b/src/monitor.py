import os,time,statistics
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx=os.open(DEV+"d3",os.O_RDONLY|os.O_NONBLOCK); buf=b""
log=open("/memory/run.log","a",buffering=1)
def L(*a): log.write(" ".join(map(str,a))+"\n")
wr("d4",0); wr("d5",0)
t0=time.time(); lasttx=0; lastlog=0
win=[]
L(f"=== monitor start {t0} ===")
while time.time()-t0<170:
    now=time.time()
    v=rd("d7")
    if v=="1": L(f"D7PULSE t={now-t0:.2f}")
    try:
        s=rd("d6")
        if s: win.append(float(s))
    except: pass
    if len(win)>200: win=win[-200:]
    try:
        c=os.read(rx,4096); buf+=c
    except BlockingIOError: pass
    while b"\n" in buf:
        l,buf=buf.split(b"\n",1)
        t=l.decode(errors="replace").strip()
        if t: L(f"RXm t={now-t0:.2f} {t!r}")
    if now-lasttx>3:
        wr("d8","BETA: stationary, observing. Stay close."); lasttx=now
    if now-lastlog>10 and win:
        L(f"M t={now-t0:.0f} s={statistics.median(win):.3f} {rd('d9')}"); lastlog=now
    time.sleep(0.02)
L("monitor end")
