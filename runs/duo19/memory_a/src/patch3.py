e=open('/bot/src/explore2.py').read()
start=e.index("    HAND=sys.argv[4]"); end=e.index("    odo()\n    d,f0,f1,reason=drive")
new='''    BB=(-3.0,1.0,-0.5,3.4)  # B explored bbox
    h=round(ctl.heading(3)/90)*90
    opts=[]
    for name,ang,dist in (('L',90,l),('F',0,f),('R',-90,r),('B',180,b)):
        if dist<0.6: continue
        a=math.radians(h+ang); nx,ny=x+CELL*math.cos(a),y+CELL*math.sin(a)
        cell=(round(nx/CELL),round(ny/CELL))
        score=0.0
        if not (BB[0]<=nx<=BB[1] and BB[2]<=ny<=BB[3]): score+=3
        if cell not in visited: score+=2
        else: score-=visited[cell]
        if name=='B': score-=2.5
        if name=='F': score+=0.3
        score+=min(dist,2.5)*0.3
        opts.append((score,name,ang))
    if not opts: opts=[(0,'B',180)]
    opts.sort(reverse=True); sc_,name,ang=opts[0]
    if ang: ctl.turn(ang)
    log('turn'+name,None,'opts=%s'%[(round(s,1),n) for s,n,a in opts])
    visited[(round(x/CELL),round(y/CELL))]=visited.get((round(x/CELL),round(y/CELL)),0)+1
'''
e=e[:start]+new+e[end:]
e=e.replace("lasttx=0; steps=0","lasttx=0; steps=0; visited={}")
e=e.replace("LOG=open(","open('/bot/src/explore2.pid','w').write(str(__import__('os').getpid()))\nLOG=open(",1)
open('/bot/src/explore2.py','w').write(e)
print('ok')
