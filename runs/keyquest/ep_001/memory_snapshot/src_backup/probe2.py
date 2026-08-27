import time
def rd(p):
    with open(f'/dev/robot/d{p}') as f: return f.readline().strip()
def wr(p,v):
    with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
wr(3,5); wr(5,5)
for i in range(12):
    print(rd(7), '| d4=',rd(4),'d1=',rd(1),'d8=',rd(8),'d2=',rd(2),'d6=',rd(6))
    time.sleep(0.5)
wr(3,0); wr(5,0)
