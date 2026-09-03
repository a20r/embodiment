import robot, time, math, json, sys

CPM=1500.0; INF=9.9
class Nav:
    def __init__(self):
        self.x=0.;self.y=0.;self.e=robot.enc();self.h=robot.heading()
        try:
            p=json.load(open('/memory/pose.json'))
            self.x,self.y=p['x'],p['y']
        except: pass
    def update(self):
        e=robot.enc(); h=robot.heading()
        d=((e[0]-self.e[0])+(e[1]-self.e[1]))/2.0/CPM
        self.e=e; self.h=h
        r=math.radians(h)
        self.x+=d*math.sin(r); self.y+=d*math.cos(r)
        return h
    def save(self):
        json.dump({'t':time.time(),'x':self.x,'y':self.y},open('/memory/pose.json','w'))
nav=Nav()
def lid(): return [x if x and x>0 else INF for x in robot.lidar()]
def chkgoal():
    st=robot.status() or ''
    if 'goal=1' in st:
        robot.motors(0,0)
        with open('/memory/GOAL_FOUND.txt','a') as f:
            f.write(json.dumps({'t':time.time(),'x':nav.x,'y':nav.y,'st':st})+'\n')
        print('!!! GOAL !!!',flush=True)
        return True
    return False
def turn(tgt):
    while True:
        h=nav.update()
        d=robot.angdiff(tgt,h)
        if abs(d)<=4: robot.motors(0,0); return
        s=max(8,min(40,abs(d)*0.8)); s=s if d>0 else -s
        robot.motors(s,-s); time.sleep(0.05)
def fwd(dist,H=None,stop=0.25):
    if H is None: H=round(nav.update()/90)*90%360
    turn(H)
    e0=sum(robot.enc())/2.
    while sum(robot.enc())/2.-e0<dist*CPM:
        h=nav.update(); L=lid()
        if chkgoal(): return 'GOAL'
        front=L[0]
        if min(L[1],L[15])<0.12: front=min(front,0.18)
        if front<stop: robot.motors(0,0); return 'wall'
        err=robot.angdiff(H,h)
        steer=max(-12,min(12,1.0*err))
        r=min(L[3],L[4],L[5]); l=min(L[11],L[12],L[13])
        if r<0.5 and l<0.5: steer+=max(-6,min(6,25*(r-l)))
        elif r<0.25: steer-=4
        elif l<0.25: steer+=4
        sp=65 if front>0.55 else 32
        robot.motors(sp+steer,sp-steer); time.sleep(0.05)
    robot.motors(0,0); return 'ok'
def report():
    h=nav.update(); L=lid()
    def b(a): return L[round(((a-h)%360)/22.5)%16]
    print('pos=%.2f,%.2f h=%.0f N=%.2f E=%.2f S=%.2f W=%.2f'%(nav.x,nav.y,h,b(0),b(90),b(180),b(270)))
    print('L',[round(v,2) for v in L])
    nav.save()
if __name__=='__main__':
    for cmd in sys.argv[1:]:
        p=cmd.split(':')
        if p[0]=='T': turn(float(p[1]))
        elif p[0]=='F':
            r=fwd(float(p[1]), float(p[2]) if len(p)>2 else None)
            print('F ->',r)
            if r=='GOAL': break
        report()
    nav.save()
