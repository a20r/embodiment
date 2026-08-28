import json
d=json.load(open('/memory/grid.json'))
vis={tuple(v) for v in map(tuple,d['visited'])}
walls=d['walls']; pos=tuple(d.get('pos',(0,0)))
xs=[c[0] for c in vis]; ys=[c[1] for c in vis]
for y in range(max(ys),min(ys)-1,-1):
    row=''
    for x in range(min(xs),max(xs)+1):
        row += '@' if (x,y)==pos else ('#' if (x,y) in vis else '.')
    print(f'{y:4d} {row}')
print('x from',min(xs),'to',max(xs),'cells',len(vis))
