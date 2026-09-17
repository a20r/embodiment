import sys, time, math, json
sys.path.insert(0,'/bot/src')
from nav import *

def explore(duration=60, logp=None):
    p=Pose(); t0=time.time(); n=0
    path=[]
    while time.time()-t0<duration:
        l=lidar()
        k=pick_dir(l, min_clear=0.28)
        h=heading()
        x,y,hh=p.update()
        if k is None or (k>2 and k<14):
            # need real turn: stop and rotate toward best
            if k is not None:
                th=(h+22.5*k)%360
                turn_to(th, tol=5, timeout=4)
        else:
            th=(h+22.5*k)%360
            steer_to(th, speed=30)
        time.sleep(0.1)
        n+=1
        if n%10==0:
            path.append((round(x,2),round(y,2),round(hh,1)))
            if logp: logp.write('%.1f %.2f %.2f %.1f | %s\n'%(time.time()-t0,x,y,hh,','.join('%.2f'%v for v in l)))
    stop()
    return p, path

if __name__=='__main__':
    dur=float(sys.argv[1]) if len(sys.argv)>1 else 60
    with open('/memory/traj.log','a') as logp:
        p,path=explore(dur, logp)
    print('pose',round(p.x,2),round(p.y,2),round(p.h,1))
    print('path',path[-8:])
    print('status',status(),'extras',extras())
    json.dump(path, open('/memory/lastpath.json','w'))
