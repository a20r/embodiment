import math
def integ(path, fires, spcol=6, hdcol=5, tickcol=1):
    rows=[]
    for ln in open(path):
        a=ln.strip().split(',')
        if len(a)<10: continue
        try: rows.append((int(a[tickcol]), float(a[hdcol]), float(a[spcol])))
        except: pass
    rows.sort()
    x=y=0.0; prev=None; hp=None; out={}
    trail=[]
    for tick,hd,sp in rows:
        if abs(sp)>2.5: sp=math.copysign(2.5,sp)
        if hp is not None and abs((hd-hp+180)%360-180)>60: hd=hp
        hp=hd
        for fk in fires:
            if abs(tick-fk)<=40 and fk not in out: out[fk]=(x,y)
        if prev is None: prev=tick; continue
        dt=(tick-prev)/100.0; prev=tick
        if dt<=0 or dt>2.0: trail.append((x,y,tick,'BREAK')); continue
        r=math.radians(hd); step=sp*dt
        x+=step*math.cos(r); y+=step*math.sin(r)
        trail.append((x,y,tick,None))
    return out,trail,(x,y)
out,tr,end=integ('/bot/src/auto8.csv',{352154:'lap1',434665:'lap2'})
print('auto8 fires:',{k:(round(v[0],1),round(v[1],1)) for k,v in out.items()},'end',(round(end[0],1),round(end[1],1)))
