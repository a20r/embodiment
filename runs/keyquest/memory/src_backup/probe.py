import time
def rd(p):
    with open(f'/dev/robot/d{p}') as f:
        return f.readline().strip()
def wr(p,v):
    with open(f'/dev/robot/d{p}','w') as f:
        f.write(str(v)+'\n')
def hdgs(n=8,dt=0.25):
    return [rd(0) for _ in range(n) if not time.sleep(dt)]
print("d3=5 d5=-5")
wr(3,5); wr(5,-5)
print(hdgs())
print("d3=-5 d5=5")
wr(3,-5); wr(5,5)
print(hdgs())
wr(3,0); wr(5,0)
