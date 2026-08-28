import select, time, math
def rd(p, t=2.0):
    for _ in range(10):
        with open(f'/dev/robot/d{p}') as f:
            r,_,_=select.select([f],[],[],t)
            s=f.readline().strip() if r else ''
        if s: return s
        time.sleep(0.15)
    return ''
def wr(p,s):
    with open(f'/dev/robot/d{p}','w') as f: f.write(s+'\n')
def h(n=5):
    vals=[]
    while len(vals)<n:
        s=rd(2)
        if s: vals.append(float(s))
        time.sleep(0.12)
    x=sum(math.cos(math.radians(v)) for v in vals); y=sum(math.sin(math.radians(v)) for v in vals)
    return round(math.degrees(math.atan2(y,x))%360,1)
for cmd in ['45','45','90','22.5','-90']:
    h0=h(); wr(5,cmd); time.sleep(0.8); h1=h()
    d=(h1-h0)%360
    if d>180: d-=360
    print(cmd, h0, h1, round(d,1))
