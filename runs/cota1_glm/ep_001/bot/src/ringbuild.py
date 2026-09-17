import math, pickle
trail=pickle.load(open('/bot/src/world_trail.pkl','rb'))
TICK_STOP=684500   # ~12m before F45 fire tick 687750 on the arrival leg
RCUT=3.5
# trail is tick-sorted; take indices from end down to tick>=TICK_STOP
idx_last=len(trail)-1
seg=[p for p in trail if p[2]>=TICK_STOP]
print('backward span points',len(seg),'from',seg[-1][2],'to',seg[0][2])
C=[(seg[-1][0],seg[-1][1])]
def d2(a,b): return (a[0]-b[0])**2+(a[1]-b[1])**2
for i in range(len(seg)-1,-1,-1):
    P=(seg[i][0],seg[i][1])
    cut=False
    for j in range(len(C)-8):
        if d2(P,C[j])<RCUT*RCUT:
            del C[j+1:]; cut=True; break
    if not cut: C.append(P)
print('after pass1:',len(C))
for _ in range(2):
    C2=[C[0]]; ncut=0
    for P in C[1:]:
        cut=False
        for j in range(len(C2)-8):
            if d2(P,C2[j])<RCUT*RCUT:
                del C2[j+1:]; ncut+=1; cut=True; break
        if not cut: C2.append(P)
    C=C2
print('after pass3:',len(C),'cuts',ncut)
L=0.0
for a,b in zip(C,C[1:]): L+=math.hypot(b[0]-a[0],b[1]-a[1])
print('ring path length %.1f m, start (%.1f,%.1f) end (%.1f,%.1f)'%(L,C[0][0],C[0][1],C[-1][0],C[-1][1]))
# resample 2m
R=[C[0]]; acc=0.0
for a,b in zip(C,C[1:]):
    d=math.hypot(b[0]-a[0],b[1]-a[1]); acc+=d
    if acc>=2.0: R.append(b); acc=0.0
if R[-1]!=C[-1]: R.append(C[-1])
print('resampled pts',len(R))
with open('/bot/src/ring.txt','w') as f:
    for px,py in R: f.write('%.2f %.2f\n'%(px,py))
pickle.dump(R,open('/bot/src/ring.pkl','wb'))
# validate: max gap between consecutive ring pts, and dist from ring end to zone
zx,zy=7.75,-107.2
print('ring end dist to zone: %.1f'%(math.hypot(R[-1][0]-zx,R[-1][1]-zy)))
gaps=[math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(R,R[1:])]
print('max gap %.2f mean %.2f'%(max(gaps),sum(gaps)/len(gaps)))
