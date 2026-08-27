from probe import rd, wr
import time
# back off from wall
wr(6,"-10"); wr(7,"-10")
time.sleep(1.5)
wr(6,"0"); wr(7,"0")
print("after back:", rd(3), rd(5), rd(1))
# measure spin rate: d6=5 d7=-5
h0=float(rd(3)); wr(6,"5"); wr(7,"-5"); time.sleep(2); wr(6,"0"); wr(7,"0")
h1=float(rd(3))
print("spin d6=5,d7=-5 2s:", h0, "->", h1)
