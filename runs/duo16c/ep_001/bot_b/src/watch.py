import time
from scene import read_line

prev = {}
for i in range(20):
    row = {}
    for p in ['d0','d3','d4','d5','d6','d9','d11']:
        row[p] = read_line(p, 0.5).replace('\n',' | ')[:50]
    d3 = row['d3']
    print(i, 'd4=%s d6=%s d9=%s d0=%s d5=%s d11=%s d3=%s' % (row['d4'],row['d6'],row['d9'],row['d0'],row['d5'],row['d11'],d3))
    time.sleep(0.5)
