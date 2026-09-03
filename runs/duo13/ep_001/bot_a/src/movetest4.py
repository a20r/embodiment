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
tests = [("d1","1.0"),("d7","1.0"),("d1","200"),("d1","-1.0"),("d7","-1.0"),("d1","fwd"),("d1","m 1.0")]
for p,v in tests:
    w(p,v); time.sleep(0.8)
    d2=r(2).split(',')
    print(f"after {p}={v}: d4={r(4)} d0={r(0)} d5={r(5)} d6={r(6)} d9={r(9)} d2=", ",".join(d2[3:8]), flush=True)
    w(p,"0")
