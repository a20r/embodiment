import os, time, select
BASE="/dev/robot/"
def w(p, s):
    try:
        fd = os.open(BASE+p, os.O_WRONLY); os.write(fd, (s+"\n").encode()); os.close(fd)
        return True
    except OSError as e: print("WERR", p, e); return False
def r(i, to=0.03):
    try: fd = os.open(BASE+f"d{i}", os.O_RDONLY|os.O_NONBLOCK)
    except OSError: return "E"
    rr,_,_ = select.select([fd],[],[],to)
    d = os.read(fd,256).decode().strip() if rr else "-"
    os.close(fd); return d
def state(tag):
    print(f"{tag}: d3={r(3)} d4={r(4)} d0={r(0)} d5={r(5)} d6={r(6)} d9={r(9)}", flush=True)
state("init")
# stop everything
w("d1","0"); w("d7","0"); time.sleep(0.5)
state("stopped")
# left wheel fwd
w("d1","100"); time.sleep(1.0); w("d1","0"); time.sleep(0.3)
state("d1=100 1s")
time.sleep(1.0)
# right wheel fwd
w("d7","100"); time.sleep(1.0); w("d7","0"); time.sleep(0.3)
state("d7=100 1s")
time.sleep(1.0)
# both fwd
w("d1","100"); w("d7","100"); time.sleep(1.0); w("d1","0"); w("d7","0"); time.sleep(0.3)
state("both=100 1s")
