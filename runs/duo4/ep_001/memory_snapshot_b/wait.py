import os, time
DEV="/dev/robot/"
def rd(p):
    with open(DEV+p) as f: return f.readline().strip()
def wr(p,v):
    with open(DEV+p,"w") as f: f.write(str(v)+"\n")
rx=os.open(DEV+"d3", os.O_RDONLY|os.O_NONBLOCK); buf=b""
log=open("/memory/run.log","a",buffering=1)
wr("d4",0); wr("d5",0)
t0=time.time(); lasttx=0; lastlog=0
import statistics
win=[]
while True:
    try:
        s=rd("d6")
        if s: win.append(float(s))
    except: pass
    if len(win)>150: win=win[-150:]
    try:
        c=os.read(rx,4096); buf+=c
    except BlockingIOError: pass
    while b"\n" in buf:
        l,buf=buf.split(b"\n",1)
        t=l.decode(errors="replace").strip()
        if t: log.write(f"RX! t={time.time()-t0:.1f} {t!r}\n")
    now=time.time()
    if now-lasttx>1.5:
        wr("d8","HELLO partner, I am stationary, come to me. Reply please."); lasttx=now
    st=rd("d9"); d7=rd("d7")
    if ("goal=" in st and "goal=0" not in st) or d7 not in ("","0"):
        log.write(f"!!! t={now-t0:.1f} d9={st} d7={d7}\n")
    if now-lastlog>5 and win:
        log.write(f"W t={now-t0:.0f} s={statistics.median(win):.3f} d7={d7} {st}\n"); lastlog=now
    time.sleep(0.02)
