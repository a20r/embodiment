from probe import rd, wr
import time
print("before:", rd(3), rd(1), rd(4))
wr(6, "1")
time.sleep(1)
print("after d6=1:", rd(3), rd(1), rd(4))
wr(6, "0")
wr(7, "1")
time.sleep(1)
print("after d7=1:", rd(3), rd(1), rd(4))
wr(7, "0")
