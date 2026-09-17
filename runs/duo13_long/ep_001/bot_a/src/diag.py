import math
from nav import Nav, CELL
nav = Nav()
nav.sync_enc()
nav.full_scan()
nav.update_pose()
start = nav.grid.key(nav.x, nav.y)
print('start cell', start, 'state', nav.grid.state(start), 'passable', nav.passable(start))
# print grid 15x15 around start
for gy in range(8, -9, -1):
    row = ''
    for gx in range(-8, 9):
        k = (start[0]+gx, start[1]+gy)
        s = nav.grid.state(k)
        ch = {'unk':'.', 'free':' ', 'occ':'#'}[s]
        if k == start: ch = 'R'
        row += ch
    print(f'{gy:+3d} {row}')
fk, fn = nav.find_frontier(start)
print('frontier', fk, fn)
print('passable count around:')
cnt = sum(1 for gx in range(-8,9) for gy in range(-8,9) if nav.passable((start[0]+gx, start[1]+gy)))
print('passable cells in 17x17:', cnt)
