# Reconstruct visited cells from run.log step lines
import sys,re
x=y=0; path=[(0,0)]
dirs={0:(1,0),90:(0,1),180:(-1,0),270:(0,-1)}
for ln in open(sys.argv[1] if len(sys.argv)>1 else '/memory/run.log'):
    m=re.match(r'step (\d+) h (\d+) F ([\d.]+) L ([\d.]+) R ([\d.]+) -> (\w+)',ln)
    if not m: continue
    h=int(m.group(2)); c=m.group(6)
    h={'left':(h+90)%360,'right':(h-90)%360,'back':(h+180)%360,'straight':h}[c]
    dx,dy=dirs[h]; x+=dx; y+=dy; path.append((x,y))
print(len(path),'cells, bbox',min(p[0] for p in path),max(p[0] for p in path),min(p[1] for p in path),max(p[1] for p in path))
print(path)
