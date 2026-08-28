from probe import rd, wr
import time
# watch d2 and d8 over time with no motion
for i in range(5):
    print(rd(0), rd(2), rd(3), rd(5), rd(8))
    time.sleep(0.5)
print("---- drive forward")
wr(6,"10"); wr(7,"10")
for i in range(6):
    print(rd(0), rd(2), rd(3), rd(5), rd(8), rd(1))
    time.sleep(0.5)
wr(6,"0"); wr(7,"0")
