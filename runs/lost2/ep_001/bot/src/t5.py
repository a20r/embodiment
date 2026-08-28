from probe import rd, wr
import time
def scan():
    return [float(x) for x in rd(1).split(",")]
h0=float(rd(3)); s0=scan()
# rotate ~+90 deg (heading increases at ~9.2deg/s at speed 5; use 10 for faster)
wr(6,"5"); wr(7,"-5")
while True:
    h=float(rd(3))
    d=(h-h0)%360
    if 85<d<180: break
wr(6,"0"); wr(7,"0")
time.sleep(0.5)
h1=float(rd(3)); s1=scan()
print("h0",h0,"h1",h1)
print("s0", s0)
print("s1", s1)
