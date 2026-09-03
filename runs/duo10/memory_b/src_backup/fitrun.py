import pickle, math
wins,wd=pickle.load(open('/tmp/fit.pkl','rb'))
def err_at(v,X,Y,model):
    ds=[(math.hypot(x-X,y-Y),d5) for x,y,d5 in v]
    lo,hi=0.1,20
    def err(L):
        s=0
        for d,d5 in ds:
            if model=='exp': p=math.exp(-d/L)
            elif model=='inv': p=1/(1+d/L)
            else: p=max(0,1-d/L)
            s+=(p-d5)**2
        return s
    for _ in range(25):
        m1,m2=lo+(hi-lo)/3,hi-(hi-lo)/3
        if err(m1)<err(m2): hi=m2
        else: lo=m1
    L=(lo+hi)/2
    return err(L),L
def fit(win,model):
    v=wd[win]
    best=None
    # coarse
    step=1.0; X0,X1,Y0,Y1=-4,14,-6,14
    for _ in range(4):
        bx=by=None
        x=X0
        while x<=X1:
            y=Y0
            while y<=Y1:
                e,L=err_at(v,x,y,model)
                if best is None or e<best[0]: best=(e,x,y,L); bx,by=x,y
                y+=step
            x+=step
        if bx is None: bx,by=best[1],best[2]
        X0,X1,Y0,Y1=best[1]-step,best[1]+step,best[2]-step,best[2]+step
        step/=4
    return best
out=open('/tmp/fit.out','w')
for model in ['exp','inv','lin']:
    for w in wd:
        e,X,Y,L=fit(w,model)
        rms=math.sqrt(e/len(wd[w]))
        out.write('%s win %s pos=(%.2f,%.2f) L=%.2f rms=%.4f\n'%(model,w,X,Y,L,rms))
        out.flush()
