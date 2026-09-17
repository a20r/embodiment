import json
m=json.load(open('/memory/map.json')); cells={tuple(json.loads(k)):v for k,v in m['cells'].items()}
pos=tuple(m['pos'])
xs=[c[0] for c in cells]; ys=[c[1] for c in cells]
# rows = x (N up), cols = y (E right)
for x in range(max(xs)+1,min(xs)-2,-1):
    top=''; mid=''
    for y in range(min(ys)-1,max(ys)+2):
        c=cells.get((x,y))
        if c is None: top+='   '; mid+='   '; continue
        top+='+'+('  ' if c['0'] else '--')
        mid+=('  ' if c['270'] else '| ')+('@' if (x,y)==pos else ('o' if (x,y)==(0,0) else '.'))
        mid=mid[:-1]+ ('@' if (x,y)==pos else ('o' if (x,y)==(0,0) else '.'))
    print(top); print(mid)
front=[(c,d) for c,w in cells.items() for d,o in w.items() if o and ((c[0]+{'0':1,'180':-1}.get(d,0)),(c[1]+{'90':1,'270':-1}.get(d,0))) not in cells]
print('pos',pos,'frontiers:',front)
