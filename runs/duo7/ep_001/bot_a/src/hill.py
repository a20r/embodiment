import mv, robot, time, json
def d5avg(n=10):
    vals=[]
    for _ in range(n):
        v=robot.readline('d5',0.3)
        if v: vals.append(float(v))
    return sum(vals)/max(1,len(vals))
def opendirs():
    h=mv.nav.update(); L=mv.lid()
    out={}
    for name,ang in (('N',0),('E',90),('S',180),('W',270)):
        i=round(((ang-h)%360)/22.5)%16
        out[ang]=L[i]
    return out
cur=d5avg()
print('start d5=%.3f pos=%.2f,%.2f'%(cur,mv.nav.x,mv.nav.y),flush=True)
lastdir=None
for step in range(40):
    if mv.chkgoal():
        print('GOAL! broadcasting')
        while True:
            robot.motors(0,0); robot.tx('GOALFOUND alpha waiting on goal')
            time.sleep(8)
    od=opendirs()
    # candidate dirs sorted: prefer continuing lastdir
    cands=[a for a,d in od.items() if d>0.42]
    if not cands: cands=[max(od,key=od.get)]
    tried=[]
    moved=False
    # order: lastdir first
    if lastdir in cands:
        cands.remove(lastdir); cands.insert(0,lastdir)
    for ang in cands:
        r=mv.fwd(0.45,ang,stop=0.22)
        if r=='GOAL':
            print('GOAL! broadcasting')
            while True:
                robot.motors(0,0); robot.tx('GOALFOUND alpha waiting on goal'); time.sleep(8)
        new=d5avg()
        print('step%d dir%d r=%s d5 %.3f->%.3f pos=%.2f,%.2f'%(step,ang,r,cur,new,mv.nav.x,mv.nav.y),flush=True)
        if new>cur+0.005:
            cur=new; lastdir=ang; moved=True; break
        elif new<cur-0.02:
            # went downhill: go back
            back=(ang+180)%360
            mv.fwd(0.45,back,stop=0.22)
            cur=d5avg()
        else:
            cur=new; lastdir=ang; moved=True; break  # flat: accept, keep exploring
    mv.nav.save()
