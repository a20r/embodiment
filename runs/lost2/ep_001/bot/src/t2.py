from probe import rd, wr
import time
print("before:", rd(3), rd(1))
wr(6,"5"); wr(7,"5")
time.sleep(2)
print("both=5:", rd(3), rd(1))
wr(6,"0"); wr(7,"0")
wr(6,"5"); wr(7,"-5")
time.sleep(1)
print("spin:", rd(3), rd(1))
wr(6,"0"); wr(7,"0")
print("d0,d2,d5,d8:", rd(0), rd(2), rd(5), rd(8))
