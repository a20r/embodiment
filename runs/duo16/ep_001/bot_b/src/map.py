import sys, json, math
pts = json.load(open('/memory/scans.json'))['C']
# grid 0.1m
def amap(pts, w=9, h=13, cx=0.0, cy=0.6):
    # x horizontal -w..w, y vertical cy-h*0.1 .. cy
    grid={}
    for x,y,z in pts:
        if z<-0.05 or z>0.15: continue
        gx=int(round((x-cx)/0.1)); gy=int(round((cy-y)/0.1))
        if abs(gx)>w or gy<0 or gy>=h: continue
        grid[(gx,gy)]=grid.get((gx,gy),0)+1
    for gy in range(h):
        row=''
        for gx in range(-w,w+1):
            c=grid.get((gx,gy),0)
            row += '.' if c==0 else ('%x'%min(c,15))
        print('%5.1f %s'%(cy-gy*0.1, row))
    print('      ' + ''.join('|' if i==w else ' ' for i in range(2*w+1)))
print('MAP of scan C (x horiz -0.9..0.9, y vert 1.2..0.0), robot assumed near (0,~0.6?)')
amap(pts)
