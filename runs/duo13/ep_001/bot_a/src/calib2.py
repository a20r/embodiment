import os, time, select
BASE="/dev/robot/"
def w(p, s):
    try:
        fd = os.open(BASE+p, os.O_WRONLY); os.write(fd, (s+"\n").encode()); os.close(fd)
    except OSError as e: print("WERR", p, e)
def r(i, to=0.03):
    try: fd = os.open(BASE+f"d{i}", os.O_RDONLY|os.O_NONBLOCK)
    except OSError: return "E"
    rr,_,_ = select.select([fd],[],[],to)
    d = os.read(fd,256).decode().strip() if rr else "-"
    os.close(fd); return d
def enc():
    a=r(6); b=r(9)
    try: return int(a),int(b)
    except: return None,None
def state(tag):
    print(f"{tag}: d3={r(3)} d4={r(4)} d0={r(0)} d5={r(5)} d11={r(11)} enc={enc()}", flush=True)
w("d1","0"); w("d7","0"); time.sleep(0.4)
state("stop")
# float test: d1=50.5
w("d1","50.5"); time.sleep(1.0); w("d1","0"); time.sleep(0.3)
state("float 50.5 (left)")
time.sleep(0.5)
# negative right
w("d7","-100"); time.sleep(1.0); w("d7","0"); time.sleep(0.3)
state("d7=-100 (right rev)")
time.sleep(0.5)
# radio hello
w("d8","HELLO ARE YOU THERE?")
time.sleep(0.5)
print("radio rx:", repr(r(10, 0.4)), flush=True)
state("after radio")
