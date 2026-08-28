import rob, walker, time, json
bng=walker.bng
# slow lap with continuous logging
f=open('/tmp/lap.jsonl','w')
t0=time.time()
def logrow():
    b=bng(); L=rob.lidar()
    f.write(json.dumps([round(time.time()-t0,2), rob.odo(), round(b,1)]+L)+"\n")
while time.time()-t0<170:
    if rob.goal(): print("GOAL!!!"); break
    walker.align()
    logrow()
    F,R,Lt,B=walker.look()
    b=bng()
    opts=[]
    for dd,dist in ((0,F),(90,R),(-90,Lt),(180,B)):
        if dist<0.38: continue
        opts.append((abs(walker.serr((b+dd)%360,90)),dd,dist))
    opts.sort()
    if not opts: walker.turn(180); continue
    _,dd,dist=opts[0]
    if dd: walker.turn(dd)
    # slow step with logging
    o0=rob.odo()
    while rob.odo()-o0<78:
        L=rob.lidar()
        if 0<L[0]<0.20: break
        r_,l_=L[4],L[12]
        c=0.0
        if 0<r_<0.45 and 0<l_<0.45: c=(r_-l_)*35
        elif 0<r_<0.28: c=(r_-0.18)*45
        elif 0<l_<0.28: c=-(l_-0.18)*45
        c=max(-7,min(7,c))
        rob.motors(12+c,12-c)
        logrow()
        time.sleep(0.05)
    rob.motors(0,0)
f.close()
print("done")
