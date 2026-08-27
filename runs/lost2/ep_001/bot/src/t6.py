from probe import rd, wr
import time
def scan(): return [float(x) for x in rd(1).split(",")]
def enc(): return (int(rd(2)), int(rd(8)))
# face open direction: currently ray0=0.83 (open). drive forward, measure encoder vs lidar
e0=enc(); s0=scan()
wr(6,"10"); wr(7,"10")
time.sleep(2.5)
wr(6,"0"); wr(7,"0")
time.sleep(0.3)
e1=enc(); s1=scan()
print("enc delta", e1[0]-e0[0], e1[1]-e0[1])
print("front", s0[0], "->", s1[0])
print(s1)
print(rd(4), rd(5))
