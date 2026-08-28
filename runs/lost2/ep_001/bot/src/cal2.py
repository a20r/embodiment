from probe import rd, wr
import time
def scan(): return [float(x) for x in rd(1).split(",")]
def enc(): return (int(rd(2)), int(rd(8)))
wr(6,"0");wr(7,"0")
# drive forward slowly until bump or front small
wr(6,"40"); wr(7,"40")
for i in range(60):
    if rd(5)=="1" or scan()[0]<0.15: break
wr(6,"0"); wr(7,"0"); time.sleep(0.3)
s0=scan(); e0=enc(); print("at wall front=",s0[0])
wr(6,"-40"); wr(7,"-40")
time.sleep(2.0)
wr(6,"0"); wr(7,"0"); time.sleep(0.3)
s1=scan(); e1=enc()
d=e1[0]-e0[0], e1[1]-e0[1]
print("front",s0[0],"->",s1[0],"enc",d)
print("units/count:",(s1[0]-s0[0])/abs(d[0]))
