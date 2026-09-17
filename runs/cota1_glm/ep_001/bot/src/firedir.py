import math, pickle
trail=pickle.load(open('/bot/src/world_trail.pkl','rb'))
def show(t0,t1,step=250):
    pts=[(x,y,t) for x,y,t in trail if t0<=t<=t1]
    print('ticks %d..%d'%(t0,t1))
    for i in range(0,len(pts),step):
        x,y,t=pts[i]
        print('  t=%d (%.1f,%.1f)'%(t,x,y))
print('=== F45 approach/departure ==='); show(683500,692500)
print('=== F34 approach/departure ==='); show(660000,668000)
