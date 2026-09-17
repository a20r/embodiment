import os, select, time, collections

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

def cap(n=3):
    for k in range(n):
        s = read('d2')
        pts = [p for p in s.split(';') if p.strip()]
        rows = []
        for p in pts:
            try: rows.append(tuple(map(float, p.split(','))))
            except: pass
        c1 = sorted(set(round(r[0],3) for r in rows))
        c2 = sorted(set(round(r[1],4) for r in rows))
        c3 = sorted(set(round(r[2],4) for r in rows))
        print(f"--- cap {k}: n={len(rows)}")
        print("col1 distinct:", len(c1), "min/max:", (c1[0], c1[-1]) if c1 else None)
        print("col2 distinct:", len(c2), c2[:10], "..." if len(c2)>10 else "")
        print("col3 distinct:", len(c3), c3[:10], "..." if len(c3)>10 else "")
        # periodicity of col3
        if len(c3) > 5:
            diffs = [round(c3[i+1]-c3[i],4) for i in range(min(20,len(c3)-1))]
            print("col3 diffs:", diffs)
        time.sleep(0.5)

cap(3)
print("d0:", read('d0'), "d5:", read('d5'), "d11:", read('d11'))
