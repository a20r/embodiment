import os, select, time

D = '/dev/robot/'
def read(p, timeout=0.25):
    fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
    r,_,_ = select.select([fd], [], [], timeout)
    out = ''
    if r:
        try: out = os.read(fd, 1000000).decode().strip()
        except Exception: out = ''
    os.close(fd)
    return out

s = read('d2')
pts = [p for p in s.split(';') if p.strip()]
rows = [tuple(map(float,p.split(','))) for p in pts]
print("n:", len(rows))
print("first 30 in order:")
for r in rows[:30]: print("  ", r)
print("...")
# check 117x24 or 156x18 grid hypothesis: smoothness within row for col2, col3
for W in (24, 18, 26, 12):
    if len(rows) % W: continue
    H = len(rows)//W
    # within-row variance of col3
    import statistics
    wr = [abs(rows[i+1][2]-rows[i][2]) for i in range(len(rows)-1) if (i+1)%W != 0]
    br = [abs(rows[i+W][2]-rows[i][2]) for i in range(len(rows)-W)]
    wr2 = [abs(rows[i+1][1]-rows[i][1]) for i in range(len(rows)-1) if (i+1)%W != 0]
    br2 = [abs(rows[i+W][1]-rows[i][1]) for i in range(len(rows)-W)]
    print(f"W={W} H={H} col3 within-row mean diff {sum(wr)/len(wr):.4f} between-row {sum(br)/len(br):.4f} | col2 within {sum(wr2)/len(wr2):.4f} between {sum(br2)/len(br2):.4f}")
