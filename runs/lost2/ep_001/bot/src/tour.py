import nav, time, math
pts=[(0.5,0.5),(0.5,2.5),(1.5,1.5),(2.6,2.6),(2.6,0.3)]
nav.upd(); print("start", nav.x, nav.y)
for tx,ty in pts:
    r=nav.goto(tx,ty,timeout=45)
    nav.upd()
    print("goto",tx,ty,"->",r,f"at ({nav.x:.2f},{nav.y:.2f})")
    nav.survey()
    print("surveyed")
    nav.save()
print("done")
