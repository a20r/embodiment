import math, time, collections
from scene import read_line as rl

def prof():
    s = rl('d2',1.0)
    pts=[tuple(map(float,t.split(','))) for t in s.split(';') if t]
    az=collections.defaultdict(list)
    for x,y,z in pts:
        r=math.hypot(x,y); a=math.atan2(y,x)
        b=int((a+math.pi)/(2*math.pi)*36)%36
        az[b].append(r)
    return {b: sorted(rs)[len(rs)//2] for b,rs in az.items() if len(rs)>5}

profs=[prof() for _ in range(5)]
for i in range(1,5):
    common = sorted(set(profs[0]) & set(profs[i]))
    diffs = [abs(profs[0][b]-profs[i][b]) for b in common]
    print(f'scan0 vs scan{i}: mean|dmed|={sum(diffs)/len(diffs):.3f} max={max(diffs):.3f} n={len(common)}')
# also raw point count jitter
